/*
 * Phase 7.4 Plan 02 Wave 1 — smart_md_handler.c
 * Phase 07.4.2 Plan 02 Wave 1 — extended to 6-motor fleet (D1+D2 LOCK).
 *
 * H7-resident smart-MD x8h7 channel handler (peripheral 0x0B). The module
 * provides:
 *
 *   1. A cached batched motor-state surface (s_state_cache[SMART_MD_MAX_MOTORS])
 *      that is kept fresh by polling FDCAN2 (the carrier-hat external CAN bus
 *      where the wheeled-biped fleet is wired per Phase 06.3-06.5 hardware
 *      lock) at 1 kHz from a HAL_TIM IRQ callback.
 *
 *   2. An SPI dispatch hook (smart_md_handler) registered via
 *      peripheral_register_callback(PERIPH_SMART_MD, ...) parallel to
 *      peripheral_register_callback(PERIPH_H7, &h7_handler) — the empirically-
 *      proven 1500 Hz-capable dispatch path per the 2026-05-07 SPI rate
 *      research memo. Decodes smart_md_command_t requests (148 B post-07.4.2 D1;
 *      was 144 B at Phase 07.4 4-motor envelope), validates header + opcode,
 *      builds a smart_md_response_t (208 B post-07.4.2 D1; was 144 B) directly
 *      from s_state_cache, and returns it via enqueue_packet (PRIMASK-safe).
 *
 * The critical invariant — locked by Phase 07.4 CONTEXT R2 (2026-05-07) and
 * the must_haves of plan 07.4-02 — is that the cache refresh runs in IRQ
 * context (HAL_TIM_PeriodElapsedCallback), NOT from main.c handle_data()'s
 * main-loop polling path. Coupling smart-MD to the main-loop NIRQ/NCS window
 * (the same window can_handler.c lives behind) would re-incur the ~30 ms
 * B_SPI2H7 penalty empirically measured on the SOCKETCAN substrate.
 *
 * NEGATIVE INVARIANTS (enforced by tests/phase07.4-smart-md-build.sh greps):
 *   - smart_md_cache_refresh_tick() is NEVER called from handle_data() in main.c.
 *   - smart_md_handler / smart_md_cache_refresh_tick NEVER call any function
 *     in can_handler.c (no can_handle_data, no fdcan1_handler, no fdcan2_handler).
 *     FDCAN2 RX FIFO is drained directly via can_read(&fdcan_2, ...) which is
 *     a thin wrapper over HAL_FDCAN_GetRxMessage (interrupt-safe per ST HAL spec).
 *     (Phase 7.4.8 patch 0043 D-01b: this ownership is now ENFORCED at runtime
 *      — see s_smart_md_active flag + smart_md_is_active() accessor below. The
 *      flag is set TRUE at the end of smart_md_handler_init(); the two-pronged
 *      DISABLE per Pitfall 3 fires once both CAN_INIT and smart_md_handler_init
 *      have completed. (a) on_CAN_INIT_Request in can_handler.c disarms the
 *      FDCAN2 RX FIFO0 IRQ notification (HAL_FDCAN_DeactivateNotification),
 *      and (b) can_handle_data()'s is_can2_init main-loop drainer is gated by
 *      !smart_md_is_active(). Both prongs are mandatory; the kernel-side
 *      PERIPH_FDCAN2 CAN_RX_FRAME sink is dormant during smart-MD operation
 *      per Plan 01 AUDIT §1 verdict no_non_md_consumers_found.)
 *   - HAL_FDCAN_Init / HAL_FDCAN_Start / HAL_FDCAN_Stop / HAL_FDCAN_DeInit
 *     are NEVER called (R-A5 carry-forward — keeps SOCKETCAN parallel deployment intact).
 *     (Patch 0043 calls HAL_FDCAN_DeactivateNotification from can_handler.c
 *      on_CAN_INIT_Request — that is NOT one of the forbidden Init/Start/Stop/DeInit
 *      functions; it is a notification-mask write that does not touch peripheral
 *      bring-up. The negative invariant is preserved.)
 *
 * Phase 07.4.2 D2 PRIMARY (pipelined FDCAN cache refresh):
 *   The TIM7 1 kHz tick acts as a pipelined RX-FIFO drain — each tick drains up
 *   to SMART_MD_MAX_MOTORS frames from FDCAN2 RX FIFO 0 into s_state_cache by
 *   matching CAN ID against s_known_md_can_ids[]. The MD80 motor fleet
 *   auto-broadcasts mainState frames on FDCAN2 when enabled (driven by the
 *   userspace OphanimSystem path via CANdle-SDK / x8h7_can substrate). Worst-
 *   case staleness for any one motor: 6 ms (one full pipeline pass at 1 kHz
 *   tick × 6 motors); average ≈ 3 ms. Acceptable for Phase 7.5 LQR ≥ 200 Hz
 *   per Wave 0 RESEARCH §3.5 LOCK rationale. The fdcan_2.State guard from
 *   patch 0026 is preserved at the top of the tick.
 */

#include "smart_md_handler.h"

#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cs8h7_smart_md_frame.h"   /* shared wire-protocol header (lives in ../include) */
#include "peripherals.h"            /* PERIPH_SMART_MD */
#include "can.h"                    /* can_read, fdcan_2 */
#include "system.h"                 /* enqueue_packet — PRIMASK-safe */
#include "debug.h"
#include "error_handler.h"

/* fdcan_2 is defined in src/can.c (FDCAN2 = carrier-hat external CAN bus
 * per Phase 06.3-06.5 hardware lock; motor 311 is wired here). Declared here
 * rather than in can.h because Wave 1 keeps the patch surface minimal. */
extern FDCAN_HandleTypeDef fdcan_2;

/*===========================================================================
 * Static state — cache + diagnostic counters
 *===========================================================================*/

/* Phase 07.4.2 D1: known motor list extended from 1 → SMART_MD_MAX_MOTORS (6)
 * to cover the wheeled-biped fleet (2× hip AK10-9, 2× knee AK10-9, 2× wheel
 * AK60-6). The exact CAN IDs are placeholder (311..316) — the operator pre-flight
 * (Wave 1 T5) confirms the fleet IDs and the bench (T6) populates s_known_md_can_ids
 * as appropriate. Future SET_MOTOR_LIST opcode (out of scope here) will let
 * userspace configure the list at runtime; Wave 1 hardcodes the placeholder
 * fleet IDs as the build-time default.
 *
 * Slots beyond SMART_MD_KNOWN_MOTOR_COUNT are zero-initialized (CAN ID 0 never
 * matches a real motor — smart_md_find_motor_index returns -1 cleanly). */
#define SMART_MD_KNOWN_MOTOR_COUNT 6
static const uint16_t s_known_md_can_ids[SMART_MD_MAX_MOTORS] = {310, 311, 312, 313, 314, 315};

/* Cache populated by smart_md_cache_refresh_tick (IRQ context) and read by
 * smart_md_handler (also IRQ-context-callable, dispatched from dma_handle_data
 * or higher). enqueue_packet PRIMASK-protects publication; the cache itself is
 * read/written behind the same PRIMASK in this file. */
static motor_state_t s_state_cache[SMART_MD_MAX_MOTORS];

/* Diagnostic drop counters (parallel to Phase 7.3-03 cs8h7_rpmsg_*_drops_* in rpc.c).
 * Exposed via SMART_MD_OP_NOOP diagnostic frame in a future plan. */
static uint32_t s_drops_bad_size       = 0;
static uint32_t s_drops_bad_opcode     = 0;
static uint32_t s_drops_bad_crc        = 0;
static uint32_t s_drops_bad_motor_count = 0;
static uint32_t s_drops_motor_not_found = 0;

/* Refresh-tick counters (incremented from IRQ context). */

/* Phase 07.4.8 patch 0044 — Layer 2 top-level H7 bus state machine state.
 *
 * Per CONTEXT D-02/D-03 + Plan 02 Task 2 interfaces: 5-state machine that
 * makes bus health a precondition for the lockstep loop. The BUS_RESETTING
 * entry replaces the broken in-lockstep BO preempt (deleted from
 * smart_md_lockstep_step by this patch) with a Bosch M_CAN AN004 §2-conformant
 * recovery: clear CCCR.INIT, then poll PSR.ACT == 1 (NOT CCCR.INIT == 0 —
 * RESEARCH §"Pitfall 1", Codex REVIEW #2). The dispatcher's BUS_HEALTHY-only
 * short-circuit guarantees smart_md_lockstep_step is unreachable when
 * PSR.BO == 1 (RESEARCH §"Pattern 3").
 *
 * State diagram:
 *   (boot) → BUS_RESETTING (CCCR.INIT cleared, TX-ring flushed, wait PSR.ACT==1)
 *     → BUS_PROBING (one READ_REGISTER per known motor, 5 ms timeout each)
 *       → BUS_HEALTHY (any motor responded) → patch 0042 runs unchanged
 *       → BUS_LINK_DOWN (no motor responded; terminal)
 *     [during BUS_HEALTHY, PSR.BO=1 detected] → BUS_OFF (exp backoff 10/50/250/1000 ms)
 *       → BUS_RESETTING (backoff elapsed)
 */
enum smart_md_bus_state {
    BUS_RESETTING   = 0,
    BUS_PROBING     = 1,
    BUS_HEALTHY     = 2,
    BUS_OFF         = 3,
    BUS_LINK_DOWN   = 4,
    BUS_STATE_COUNT = 5
};
static enum smart_md_bus_state s_bus_state         = BUS_RESETTING;
static uint32_t                s_bus_state_enter_ms = 0;
/* Phase 07.4.15 Plan 02 patch 0062 — cold-boot bus state machine init fix.
 *
 * SYMPTOM (Plan 01 Wave-2A evidence, runs/phase07.4.15-forensic/20260516T231131Z/):
 *   - FDCAN2.CCCR.INIT = 1 (controller held in INIT mode) post-bench
 *   - FDCAN2.PSR.ACT = 0 (Synchronizing — never reaches Idle while INIT=1)
 *   - s_bus_state_transitions[BUS_RESETTING] = 0 (entry routine never called)
 *   - s_md_writes_emitted = 0 during entire activate window
 *   - launch.log: all smart-MD writes returned status=0x0B BUS_NOT_READY
 *
 * ROOT CAUSE:
 *   The static initializer above (s_bus_state = BUS_RESETTING) skips
 *   bus_enter_RESETTING() — which is THE routine that toggles CCCR.INIT
 *   (clear → set → clear, AN004 §2 conformant). On cold boot the state
 *   machine ticks into bus_state_tick_in_RESETTING() and polls PSR.ACT==1,
 *   but PSR.ACT can never reach Idle while CCCR.INIT=1, deadlocking the
 *   state machine. The 20 ms guard timer SHOULD fall through to BUS_OFF —
 *   but s_bus_state_transitions[BUS_OFF] = 0 confirms the tick path itself
 *   never observed sufficient elapsed time, OR the static-init of
 *   s_bus_state_enter_ms = 0 makes (HAL_GetTick() - 0) appear "fresh".
 *
 * FIX (option a per Plan 02 prompt — minimal blast radius):
 *   On the FIRST tick of smart_md_bus_state_tick() after fdcan_2.Instance
 *   becomes valid (i.e. after on_CAN_INIT_Request fires for FDCAN2),
 *   force a single explicit bus_enter_RESETTING() invocation. This:
 *     (a) clears latched error IR bits (BO/EP/EW)
 *     (b) flushes the TX ring (head=tail at cold boot, harmless)
 *     (c) toggles CCCR.INIT (set then clear) per AN004 §2
 *     (d) increments s_bo_recovery_attempts AND s_bus_state_transitions[BUS_RESETTING]
 *     (e) re-arms s_bus_state_enter_ms to HAL_GetTick() — so the 20 ms
 *         guard timer in tick_in_RESETTING() resumes from a known epoch
 *
 * The one-shot flag is BSS-init false; the cold-init runs exactly once per
 * H7 boot. Subsequent bus-off / recovery cycles re-enter RESETTING via
 * bus_state_tick_in_OFF() -> bus_enter_RESETTING() (unchanged path).
 *
 * The new counter s_cold_boot_init_fix_invocations is a falsifier: post-fix
 * it MUST read 1 (incremented on the single cold-init invocation). If it
 * reads 0 post-bench, the cold-init path didn't run (different deadlock).
 * If it reads >1, the one-shot flag isn't holding (logic bug). */
static volatile bool     s_cold_init_done                = false;
static volatile uint32_t s_cold_boot_init_fix_invocations = 0;
/* Per-state entry counters; capped at 255 (8-bit per slot in the packed
 * diag encoding — RESEARCH §"Pattern 5"). T6 60 s soak cannot realistically
 * exceed ~60 BUS_RESETTING entries even on a pathological bus. */
static uint8_t                 s_bus_state_transitions[BUS_STATE_COUNT] = {0};
/* BUS_OFF exponential-backoff (10 → 50 → 250 → 1000 ms cap per D-02d). */
static uint32_t                s_bo_backoff_ms     = 10u;
static uint32_t                s_bo_off_enter_ms   = 0;
/* BUS_PROBING cursor (one motor per tick, 5 ms each). */
static uint8_t                 s_bus_probe_motor_idx = 0;
static uint8_t                 s_bus_probe_responses = 0;
/* Phase 07.4.8 patch 0046 — BO-trigger reason instrumentation.
 *
 *   s_last_bo_trigger_reason: 4-bit enum (last reason BUS_HEALTHY/PROBING
 *     transitioned to BUS_OFF/BUS_LINK_DOWN). Discriminates the three
 *     Phase 7.4.8 PARTIAL candidate root causes via the GET_DIAG dump.
 *   s_bus_probe_responses_total: lifetime saturating count of probe
 *     READ_REGISTER responses received across all BUS_PROBING passes.
 *     == 0 -> probes never see a single MD80 reply (cause 2 strongest).
 *      > 0 -> probes occasionally succeed (cause 1 or 3 in BUS_HEALTHY).
 */
#define BO_REASON_NONE                    0u
#define BO_REASON_PSR_BO_BIT              1u
#define BO_REASON_PROBING_ZERO_RESPONSES  2u
static uint8_t                 s_last_bo_trigger_reason    = BO_REASON_NONE;
static uint8_t                 s_bus_probe_responses_total = 0u;
/* Phase 07.4.9 patch 0047 — seed absent_mask from BUS_PROBING responders.
 * Closes lockstep NACK-storm on absent motor slots per
 * runs/phase07.4.7-rate-ceiling/20260514T153612Z-phase07.4.8-instrumented-T1-single-motor/timeout_1000us/T1-diagnosis.md cause (3).
 * H7-internal only — no Linux schema change (CONTEXT D-04d remains deferred). */
#define SMART_MD_KNOWN_MOTOR_VALID_MASK   ((uint8_t)((1u << SMART_MD_MAX_MOTORS) - 1u))  /* 0x3F */
static uint8_t                 s_motor_absent_mask             = 0u;
static uint8_t                 s_bus_probe_responders_bitmask  = 0u;
/* BUS_RESETTING entry timestamp for the 20 ms PSR.ACT guard timer. */
#define SMART_MD_BUS_RESETTING_GUARD_MS 20u
#define SMART_MD_BUS_PROBING_TIMEOUT_US 5000u   /* 5 ms per motor (D-02c) */
#define SMART_MD_BO_BACKOFF_INITIAL_MS  10u
#define SMART_MD_BO_BACKOFF_GROWTH      5u
#define SMART_MD_BO_BACKOFF_MAX_MS      1000u
#define SMART_MD_BO_QUIET_REATTEMPT_MS  5000u   /* "quiet 5 s" predicate per D-02d */

/* Phase 07.4.8 patch 0043 — exclusive FDCAN2 RX FIFO0 ownership flag (D-01b/c).
 *
 * Per Plan 01 AUDIT verdict `no_non_md_consumers_found` (Layer 1 DISABLE branch
 * per CONTEXT D-01b): smart_md owns the FDCAN2 RX FIFO0 once initialised. The
 * flag is set TRUE at the end of smart_md_handler_init() so that BOTH (a) the
 * main-loop drainer in can_handle_data()::is_can2_init branch AND (b) the
 * runtime FDCAN2 IRQ notification (deactivated in on_CAN_INIT_Request when the
 * flag is true) skip handling FDCAN2 RX frames. Pitfall 3: ONLY landing one
 * prong reproduces the Phase 7.4.6 stolen-response symptom shape — both must
 * fire (CONTEXT.md D-01c; RESEARCH §"Pitfall 3"; Plan 01 AUDIT §6).
 *
 * Lifecycle per RESEARCH OQ-1 option (a): flag is monotonic — flipped TRUE
 * exactly once at the end of smart_md_handler_init() (before the FDCAN2
 * CAN_INIT opcode arrives over SPI), never cleared. The IRQ disarm happens
 * later at on_CAN_INIT_Request time once the bus is actually initialised
 * (can_init re-arms the RX notification, so deactivate AFTER is_can2_init=true).
 * No race because smart_md_handler_init runs in peripheral_init() before
 * gpio_init_ncs() — Linux→H7 SPI dispatch cannot yet deliver CAN_INIT. */
static bool s_smart_md_active = false;

bool smart_md_is_active(void)
{
  return s_smart_md_active;
}

/* Phase 07.4.3 Plan 02 Wave 1 — write-side opcode counters.
 * Mirror the s_drops_* family pattern: parallel diagnostic visibility for the
 * outbound smart_md_emit_md_write helper. Surfaced via dmesg or a future
 * sysfs-readback opcode (out of scope here; Phase 7.5 backlog item). */
static uint32_t s_md_writes_emitted          = 0;
static uint32_t s_md_writes_tx_full          = 0;
static uint32_t s_md_writes_fdcan_not_ready  = 0;
static uint32_t s_md_writes_torque_clamped   = 0;  /* T-07.4.3-01 mitigation */
/* Phase 07.4.11 (D-02): counts DUAL builder invocations — G13 verification counter */
static uint32_t s_md_writes_frame_kind_one = 0;
/* Phase 07.4.12 (D-03): CF-1 no-ACK-on-DUAL-frame counter — embedded-systems
 * REVIEW CF-1 + rtos-engineer REVIEW RT-3 falsifier. Increments when a no-ACK
 * timeout fires AND the timed-out slot has frame_kind == SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL.
 * If > 0 after the activate sequence, patch 0049's consumer-side reset is
 * insufficient — the no-ACK stall branch ALSO needs a reset (Phase 7.4.13 D-06
 * fix candidate). */
static uint32_t s_md_writes_no_ack_at_frame_kind_one = 0;
/* Phase 07.4.12 (D-04): RT-2 plausibility latency probe — SET_IMPEDANCE_GAINS
 * opcode handler duration in microseconds, captured via DWT cycle delta around
 * the case body (SYSCLK=400 MHz → / DWT_CYCLES_PER_US). If > 10000 (10 ms), the handler holds
 * PRIMASK=1 long enough to credibly create the NIRQ-stall window per RT-2. Last
 * assignment wins across multiple calls; activate sequence calls SET_IMPEDANCE_GAINS
 * once per session so the captured value reflects activate-time measurement.
 * volatile: write-only variable — prevents compiler from eliminating the store
 * as dead code (openocd reads the value directly from the BSS address). */
static volatile uint32_t s_set_impedance_gains_handler_us = 0;
/* Phase 07.4.14 patch 0058 (A3): pre-fix-window falsifier for the reg_addr
 * matcher tightening. Increments when the matcher rejects a frame because
 * resp_addr != reg_addr (and reg_addr != 0u). Post-fix the WRITE-ACK call site
 * passes slot->reg_addr; this counter quantifies how often unrelated frames
 * arrive during the WRITE-ACK window. Informational. */
static volatile uint32_t s_write_ack_reg_addr_mismatch = 0;
/* Phase 07.4.17 patch 0067 (D-07): RT-2 per-IOCTL NIRQ-stuck falsifier counters.
 * Each counter increments inside its handler's case-body drain epilogue when
 * NIRQ is found asserted (LOW) at handler exit. Pre-fix expectation per Phase
 * 7.4.16 walking-deeper evidence: at least one of these > 0 (SET_MODE
 * cycle-1, SET_TARGETS cycle-2, ENABLE cycle-3 likely offenders). Post-fix
 * expectation: all 4 -> 0 (or low-single-digit residual). Aggregate
 * s_nirq_already_low_at_handshake (declared in main.c) cross-checks the total.
 * volatile: write-only -- prevents DCE; openocd reads BSS address directly. */
static volatile uint32_t s_nirq_stuck_at_set_mode       = 0;
static volatile uint32_t s_nirq_stuck_at_set_imp_gains  = 0;
static volatile uint32_t s_nirq_stuck_at_set_targets    = 0;
static volatile uint32_t s_nirq_stuck_at_enable         = 0;
/* Phase 07.4.19 patch 0069 (D-05): DISABLE drain epilogue counter (OQ-1
 * carry-forward from Phase 7.4.17 RESEARCH Open Question #1). Increments
 * when NIRQ is found asserted at DISABLE handler exit. Informational:
 * DISABLE is a "light" handler (does not busywait); > 0 indicates residual
 * NIRQ state from a prior ENABLE/SET_TARGETS/SET_IMPEDANCE_GAINS sequence. */
static volatile uint32_t s_nirq_stuck_at_disable        = 0;
/* Phase 07.4.20 patch 0070 (D-02) -- FDCAN2 TX FIFO emission instrumentation.
 * Placed after s_nirq_stuck_at_disable (Phase 7.4.19 patch 0069) for openocd
 * readback locality. 5 BSS counters, 20 bytes total.
 * All declared static volatile uint32_t per Phase 7.4.17 RESEARCH A2 (aligned
 * 32-bit LDR/STR is atomic on Cortex-M7; single-writer/openocd-reader). */
static volatile uint32_t s_fdcan2_tx_attempt_count     = 0;  /* pre-can_write dispatch */
static volatile uint32_t s_fdcan2_tx_success_count     = 0;  /* can_write returned 0 */
static volatile uint32_t s_fdcan2_tx_fail_full_count   = 0;  /* TX FIFO full */
static volatile uint32_t s_fdcan2_tx_fail_busoff_count = 0;  /* bus state != HEALTHY at dispatch */
static volatile uint32_t s_fdcan2_tx_busy_at_activate  = 0;  /* one-shot: encoded bus state at first block */
/* Phase 07.4.20 patch 0071 (Strategy E -- write_ack_reg_addr_validation_fix).
 * Falsifier for patch 0063's 0xA0 RESPONSE_LEGACY bypass in dwt_rx_match_or_timeout.
 * Increments each time rx_buf[0]==0xA0 AND reg_addr!=0u (i.e., a WRITE-ACK match
 * attempt bypasses the reg_addr comparison because the motor returned a RESPONSE_LEGACY
 * frame). Post-0071 flash: counter should grow at >= s_md_writes_emitted rate during
 * activate (one bypass per successful WRITE+ACK exchange).
 * Pre-0063-ELF: counter stays 0 (the bypass code was not compiled in). */
static volatile uint32_t s_write_ack_0xa0_bypassed = 0;
/* PHASE_0072_NIRQ_FALSIFIERS: 5 counters added -- see Plan 02 D-04.
 * Phase 07.4.21 patch 0072 (D-04) -- SET_TARGETS NIRQ handshake falsifier
 * counters. Settles CONTEXT D-02 falsifier F-03 empirically; the 5th counter
 * (s_set_targets_at_poll_phase) mitigates RESEARCH Pitfall 1 by sampling the
 * lockstep cycle parity at SET_TARGETS entry. Symptom counter
 * s_nirq_stuck_at_set_targets (patch 0067) is preserved unchanged.
 * All declared static volatile uint32_t (R-A2: aligned 32-bit LDR/STR atomic on
 * Cortex-M7; single-writer/openocd-reader pattern). */
static volatile uint32_t s_set_targets_nirq_attempts          = 0;
static volatile uint32_t s_set_targets_nirq_succeeded         = 0;
static volatile uint32_t s_poll_active_at_set_targets_failure = 0;
static volatile uint32_t s_writeack_payload_nonzero_count     = 0;
static volatile uint32_t s_set_targets_at_poll_phase          = 0;
/* PHASE_0073_H4_DISAMBIG: 5 H4/H3 disambiguation counters + last-completed
 * handler enum + 1 optional QUICK_STATUS diagnostic counter. Disambiguates
 * H4 (prior heavy handler -- most likely SET_IMP_GAINS' PRIMASK=1 region --
 * leaves NIRQ asserted on entry to next SPI dispatch) vs H2 (ENABLE bleed)
 * vs H3 (FDCAN2 RX FIFO occupancy stall). Seed evidence:
 * s_nirq_already_low_at_handshake=236 from Phase 7.4.21 Plan 02 snapshot.
 * All declared static volatile (R-A2: aligned 32-bit / 8-bit LDR/STR atomic
 * on Cortex-M7; single-writer/openocd-reader pattern); serialization
 * provided by dma_handle_data PRIMASK=1 (set at system.c:442). */
static volatile uint32_t s_set_targets_nirq_already_low_on_entry   = 0;
static volatile uint32_t s_set_targets_handshake_retry_count       = 0;
static volatile uint32_t s_fdcan2_rx_fifo_occupancy_at_set_targets = 0;
static volatile uint8_t  s_prior_handler_at_set_targets_stuck      = 0;
static volatile uint8_t  s_last_completed_handler                  = 0;
static volatile uint8_t  s_in_set_targets_dispatch                 = 0;
static volatile uint32_t s_writeack_quick_status_nonzero_count     = 0;
enum {
    SMART_MD_LAST_HANDLER_NONE          = 0,
    SMART_MD_LAST_HANDLER_SET_MODE      = 1,
    SMART_MD_LAST_HANDLER_SET_IMP_GAINS = 2,
    SMART_MD_LAST_HANDLER_ENABLE        = 3,
    SMART_MD_LAST_HANDLER_DISABLE       = 4,
    SMART_MD_LAST_HANDLER_SET_TARGETS   = 5,
};
/* PHASE_0075_WATCHDOG_REENABLE: criterion-7 sub-check (a) counter. */
static volatile uint32_t s_watchdog_re_enable_emitted = 0;
/* PHASE_0076_H5_INTRA_BURST: sample-to-sample SET_TARGETS timing + frame-
 * position histogram. Disambiguates H5 (intra-burst SET_TARGETS-on-
 * SET_TARGETS stuck-on-entry mechanism: a prior SET_TARGETS' final
 * WRITE-ACK may leave NIRQ low if the next dispatch arrives before the
 * ACK chain settles). All declared static volatile (R-A2: aligned 32-bit
 * / 8-bit LDR/STR atomic on Cortex-M7; single-writer/openocd-reader
 * pattern); serialization provided by dma_handle_data PRIMASK=1
 * (set at system.c:442). DWT->CYCCNT wraps every ~10.7 s @ 400 MHz;
 * unsigned subtraction handles wrap safely for delta < 2^31 cycles
 * (~5.4 s). At 500 Hz io_loop_ expected delta is ~2 ms = 800,000 cycles
 * (well below wrap threshold); a defensive `delta < 2e9` sanity gate
 * skips first-sample / post-uninit reads (Pitfall 3). */
static volatile uint32_t s_set_targets_total_dispatches                  = 0;
static volatile uint32_t s_set_targets_prev_exit_dwt_cycles              = 0;
static volatile uint32_t s_set_targets_intra_burst_delta_dwt_cycles_min  = 0;
static volatile uint32_t s_set_targets_intra_burst_delta_dwt_cycles_max  = 0;
static volatile uint32_t s_set_targets_intra_burst_delta_dwt_cycles_p50_ewma = 0;
static volatile uint8_t  s_set_targets_current_frame_position            = 0xFFu;
static volatile uint8_t  s_set_targets_prev_burst_frame_position         = 0xFFu;
static volatile uint32_t s_set_targets_prev_burst_frame_position_pos     = 0;
static volatile uint32_t s_set_targets_prev_burst_frame_position_vel     = 0;
static volatile uint32_t s_set_targets_prev_burst_frame_position_tq      = 0;
/* PHASE_0077_H5_ACK_BARRIER: counts bounded spin-wait iterations at
 * SET_TARGETS exit. Diagnostic: per-call ratio = total_spins / total_
 * dispatches; expected << 10. T-7.4.23-01 IWDG mitigation: iteration-
 * count-bounded (cumulative worst-case 500 Hz x 30 s x 50 cyc = 1.88 ms
 * = 0.2% of IWDG period). */
static volatile uint32_t s_set_targets_ack_barrier_spins                 = 0;

/* Phase 7.4.24 patch 0078: SET_IMPEDANCE_GAINS exit-timestamp anchor.
 * Stamped at the SET_IMPEDANCE_GAINS handler tail AFTER the patch-0067
 * drain-on-exit and BEFORE the latency probe (Assumption A3). Plan 02
 * analyzer computes delta(next_handshake - this_anchor) for H6b
 * architectural-delay calibration. PHASE_0078_H6_SUBEVENT */
static volatile uint32_t s_set_imp_gains_exit_dwt_cycles                 = 0;

/* Phase 7.4.24 patch 0079 candidate C (chain 3c): NIRQ liveness re-arm.
 * Counts the number of times the TIM7 1-Hz callback force-raised NIRQ
 * because it had been asserted (held LOW) for > 100 ms with a tx_packet
 * still pending. Criterion-7 sub-check (b) bounded: > 0 (patch fired)
 * AND < s_nirq_gate_total_calls (no livelock; T-7.4.24-01 sanity).
 * Source-of-truth for race-safe read of s_nirq_high_at_cycles is main.c:78.
 * PHASE_0079C_TIM7_NIRQ_REARM */
extern volatile uint32_t s_nirq_high_at_cycles;
static volatile uint32_t s_nirq_liveness_rearm_count                     = 0;

/* Phase 07.4.18 patch 0068 (D-02 / D-04): C2 producer-consumer race falsifier counters.
 * Forensic-only -- no fix to the race lands here (per CONTEXT D-03; fix deferred to Phase 7.4.19).
 *
 * s_frame_kind_torn_read_count:        consumer reads a frame_kind value outside the valid
 *                                       enum range {0=REG_SINGLE, 1=IMPEDANCE_DUAL}. Belt-and-
 *                                       suspenders -- Cortex-M7 LDRB on naturally-aligned uint8_t
 *                                       is single-cycle atomic, so partial-byte tears are
 *                                       impossible. > 0 would indicate BSS corruption.
 * s_frame_kind_producer_writes:        REG_SINGLE enqueue count via smart_md_emit_md_write (P1).
 *                                       Does NOT write frame_kind explicitly -- relies on BSS zero
 *                                       (first use) or consumer-side reset at line 1015 (reuse).
 * s_frame_kind_consumer_reads:         every consumer dispatch entry in smart_md_do_write_lockstep.
 * s_frame_kind_consumer_dual_reads:    consumer sees frame_kind == IMPEDANCE_DUAL (stale-DUAL
 *                                       signal per RESEARCH OQ-2 + Pitfall 2). Differential with
 *                                       producer DUAL writes (existing s_md_writes_frame_kind_one
 *                                       at line 259) detects stale-DUAL reads on fresh boots.
 *
 * Atomicity: plain `static volatile uint32_t s_*` per Phase 7.4.17 RESEARCH A2 + Pitfall 1.
 * Producer increment runs inside PRIMASK=1 (dma_handle_data inheritance); consumer increment
 * runs in thread mode between PRIMASK=1 windows. No concurrent writer for either site ->
 * no __atomic_* or data-memory-barrier needed for the COUNTERS themselves. */
static volatile uint32_t s_frame_kind_torn_read_count    = 0;
static volatile uint32_t s_frame_kind_producer_writes    = 0;
static volatile uint32_t s_frame_kind_consumer_reads     = 0;
static volatile uint32_t s_frame_kind_consumer_dual_reads = 0;

/* Phase 07.4.15 patch 0060 — P1 wire-format forensic counters (sparse).
 * Per CONTEXT D-04 / D-07 / RESEARCH section 2.1-2.3.
 * Counter-write site lives INSIDE dwt_rx_match_or_timeout, BEFORE the
 * frame_kind + reg_addr filter checks. Counts what H7 RECEIVES regardless
 * of matcher acceptance — disambiguates "matcher rejected" from "FDCAN2
 * never delivered". */

/* Frame-kind receive counters (sparse: 0xA0 RESPONSE_LEGACY, 0x41 READ_REGISTER,
 * 0x42 WRITE_REGISTER, plus "other"). 16 B BSS. */
static volatile uint32_t s_md_rx_first_byte_A0    = 0;
static volatile uint32_t s_md_rx_first_byte_41    = 0;
static volatile uint32_t s_md_rx_first_byte_42    = 0;
static volatile uint32_t s_md_rx_first_byte_other = 0;

/* Reg-addr LOW-byte receive counters (sparse: known interesting values from
 * CANdle-SDK md_types.hpp). 40 B BSS. */
static volatile uint32_t s_md_rx_reg_addr_lo_50    = 0;  /* Kp / write of 0x050 */
static volatile uint32_t s_md_rx_reg_addr_lo_51    = 0;  /* Kd / write of 0x051 */
static volatile uint32_t s_md_rx_reg_addr_lo_62    = 0;  /* poll velocity */
static volatile uint32_t s_md_rx_reg_addr_lo_63    = 0;  /* poll position */
static volatile uint32_t s_md_rx_reg_addr_lo_64    = 0;  /* poll torque */
static volatile uint32_t s_md_rx_reg_addr_lo_40    = 0;  /* 0x040 or 0x140 */
static volatile uint32_t s_md_rx_reg_addr_lo_41    = 0;  /* 0x141 motionModeStatus */
static volatile uint32_t s_md_rx_reg_addr_lo_42    = 0;  /* 0x142 state-enable */
static volatile uint32_t s_md_rx_reg_addr_lo_00    = 0;  /* zero placeholder / status */
static volatile uint32_t s_md_rx_reg_addr_lo_other = 0;

/* 16-slot ring of last-received 8-byte frame payloads.
 * Read post-bench via openocd `mdw <ring_addr> 32` (32 words = 128 bytes = 16x8).
 * Disambiguates "FDCAN2 dropped it" (ring empty) from "matcher rejected"
 * (ring populated, counters > 0). 136 B BSS. */
static volatile uint8_t  s_md_rx_ring[16][8]      = {{0}};
static volatile uint8_t  s_md_rx_ring_head        = 0;
static volatile uint32_t s_md_rx_ring_total       = 0;
/* Phase 07.4.3 Tier 0 (patch 0029) — response-wait diagnostic counters.
 * REMOVED in Phase 07.4.5 (patch 0035): static counters retired since
 * patch 0032 removed the response-wait loop that populated them. The
 * corresponding diag-payload slots now carry FDCAN2->PSR / FDCAN2->ECR
 * snapshots (see GET_DIAG handler). */

/* Phase 07.4.6 (patch 0038) — active poll state machine.
 * MD80 protocol does not auto-broadcast; the only way to populate
 * s_state_cache from the H7 is to actively issue READ_REGISTER bulk-reads
 * for each motor in round-robin and parse the RESPONSE_LEGACY (0xA0)
 * frames that come back.  CANdle-SDK's readMainState() does exactly this
 * (MD.hpp:264-270 + 412-446); we mirror the pattern inside the H7
 * cache_refresh_tick so ros2_control's read() path sees fresh data.
 *
 * Three contiguous f32 registers per motor (mainEncoderVelocity 0x062,
 * mainEncoderPosition 0x063, motorTorque 0x064).  Bulk-read frame is
 * [0x41, 0x00, addr_lo, addr_hi, 0,0,0,0] × 3 = 20 bytes payload,
 * which fits in FDCAN-FD DLC=20.  Response from motor mirrors the
 * register-block layout: [hdr, hdr, addr_lo, addr_hi, val_b0..b3] × 3. */
#define SMART_MD_POLL_REG_COUNT 3u
static const uint16_t s_poll_reg_addrs[SMART_MD_POLL_REG_COUNT] = {
    0x062,  /* mainEncoderVelocity (f32) */
    0x063,  /* mainEncoderPosition (f32) */
    0x064,  /* motorTorque         (f32) */
};
static uint8_t  s_poll_motor_idx        = 0;    /* round-robin cursor */
static uint32_t s_poll_emitted_total    = 0;    /* RD requests TX'd OK */
static uint32_t s_poll_responses_drained= 0;    /* matching responses parsed */
__attribute__((unused))
static uint32_t s_poll_match_failures   = 0;    /* RX frame from known motor but bad format (lockstep-era: reserved) */

/* Phase 07.4.6 v3 (2026-05-12) — H7-side bus-off recovery.
 *
 * Empirical: under sustained SET_TARGETS bursts during sine motion,
 * motor 311's ACK rate dips, FDCAN2 TEC climbs past 256 and the
 * controller enters bus-off (PSR.BO=1).  Without intervention, the
 * controller would auto-recover via the 128*11-recessive-bits protocol
 * mechanism IF the bus stays quiet AND auto-retransmission has been
 * disabled.  But our config has AutoRetransmission=ENABLE (can.c init),
 * and the dispatch ring + active poll keep adding TX requests — so
 * pending TXs reload TEC each time the controller emerges from BO,
 * locking us into the BO state.
 *
 * Recovery strategy: when PSR.BO=1 is detected at the top of
 * cache_refresh_tick, toggle CCCR.INIT (raw register write) to force
 * the controller into Init mode and back out.  Per STM32H7 RM56.4.8,
 * this clears TEC/REC to 0, flushes the TX FIFO, and restarts protocol
 * operation.  Bit-timing + filter config in MessageRAM persist across
 * INIT toggle since we don't touch CCCR.CCE.
 *
 * Build-gate compliance: the gate forbids HAL_FDCAN_(Init|Start|Stop|
 * DeInit) function calls in patch 0038's added lines.  Direct register
 * writes to fdcan_2.Instance->CCCR / TXBCR / IR are NOT forbidden and
 * accomplish the same goal without HAL recursion or state-machine
 * interference.
 *
 * Throttle: at most one recovery attempt per the deprecated min-interval
 * throttle (deleted by patch 0044 — see below).  Each successful toggle
 * increments s_bo_recovery_attempts.
 *
 * Phase 07.4.8 patch 0044: the deprecated BO-recovery throttle constant and the
 * in-lockstep BO preempt at smart_md_lockstep_step() (formerly lines 716-731)
 * are DELETED. The new Layer 2 bus state machine owns BO recovery via the
 * BUS_RESETTING entry (AN004 §2 PSR.ACT==1 poll, NOT CCCR.INIT spin — Pitfall 1).
 * `s_bo_recovery_attempts` and `s_bo_recovery_last_ms` survive: `bus_enter_RESETTING`
 * increments `s_bo_recovery_attempts` and stamps `s_bo_recovery_last_ms` so G3
 * verification (smart_md_errors / BO-recovery delta) continues to work. */
static uint32_t s_bo_recovery_attempts  = 0;    /* count of bus_enter_RESETTING calls (CCCR.INIT toggles) */
static uint32_t s_bo_recovery_last_ms   = 0;    /* HAL_GetTick() at last attempt; 0 = never */

/* Phase 07.4.6 (patch 0038) — TIM7 tick prescaler for poll emit.
 *
 * Empirical finding (2026-05-11 bench, post-flash diag): firing one poll
 * per TIM7 tick (1 kHz total fleet rate) overwhelms MD80's RX state
 * machine.  Motor 311 responds to single cansend READ_REGISTER frames
 * cleanly (TEC stays 0), but bursts at 1 kHz cause unacked TXs, TEC
 * climbs by 8 per missed-ACK frame, and FDCAN2 enters bus-off in ~30
 * frames (<32 ms wall-clock).  Without an H7-side BUS_OFF recovery,
 * polls then never resume.
 *
 * Mitigation: emit only on every Nth TIM7 tick.  N=10 → 100 Hz total
 * fleet poll rate → with 6 motors round-robin, each motor refreshes
 * at ~17 Hz / ~60 ms cycle.  This is comfortable headroom for
 * ros2_control's 200 Hz outer-loop read() — joint_states position will
 * be at worst 60 ms stale, which is fine for the Plan 08 rate-sweep
 * baseline.  After Plan 08 we can revisit if a tighter cache is needed
 * (likely once the upstream Plan 09 controller-rate study lands).
 *
 * Phase 07.4.6 v2 (2026-05-12): bumped from 10 -> 100.  Empirical bench
 * with PRESCALER=10 showed combined 200 Hz SET_TARGETS + 17 Hz polls
 * pushes FDCAN2 to bus-off in <500 ms even with motor 311 in good
 * standing.  Motor accepts each path individually at full rate but the
 * combined arrival rate exceeds its ACK-buffer.  PRESCALER=100 -> 10 Hz
 * total fleet -> ~1.7 Hz per motor when mask is set.  joint_states then
 * updates at ~1 Hz, adequate for visualisation; real-time controllers
 * (LQR off IMU, not motor encoders) don't depend on this cadence.
 *
 * Phase 07.4.4 (2026-05-12 post-Plan-08): reverted to 10. The Plan 08
 * skip-unchanged gate (host-side write_smart_md_) collapses SET_TARGETS
 * from 200 Hz to ~45 fps total fleet -- per-motor combined arrival at
 * PRESCALER=10 / N=6 is now ~41 fps/motor, ~5x under the 7.4.6 v2 cliff
 * (~217 fps/motor) and ~12x under the historical CANDLE-substrate
 * production rate (500 fps/motor). Yields ~17 polls/motor/sec at N=6
 * -> ~60 ms worst-case cache age, ~40 ms p99 target met. Wave 2 bench
 * validates empirically; fallback to PRESCALER=20 if TEC climbs >50 or
 * s_bo_recovery_attempts > 0 over 60 s sine.
 */
/* Phase 07.4.5 (patch 0040): PRESCALER concept RETIRED.
 *   Lockstep paces per handle_data() iteration naturally — no software prescaler.
 *   See smart_md_lockstep_step() / s_lockstep_cycle_counter for the replacement. */

/* Phase 07.4.6 (patch 0038) — per-motor active mask.
 *
 * Empirical 2026-05-11 bench finding: round-robin polling ALL 6 fleet
 * motor IDs {310..315} when only motor 311 is actually on the bench
 * fixture causes 5/6 polls to receive no ACK (motors 310/312/313/314/315
 * absent) -> TEC bumps 8/frame -> bus-off in ~30 frames even at the
 * 100 Hz prescaled rate.
 *
 * Mitigation: only poll motors that userspace has marked active via
 * SMART_MD_OP_ENABLE.  Mask bit i corresponds to s_state_cache[i] /
 * s_known_md_can_ids[i].  Bits set on ENABLE; cleared on DISABLE.
 * Bootstrap: at boot, mask = 0 -> no polls fire, bus stays quiet.
 * ros2_control's on_activate sends ENABLE for each configured motor
 * -> only those motors are polled.  Aligns the poll set with the live
 * motor set without requiring a separate SET_POLL_LIST opcode. */
static uint8_t s_poll_active_mask = 0;  /* bit i = motor i is poll-active */

/* Phase 07.4.4 (patch 0034) — deferred-TX-dispatch ring buffer.
 * smart_md_handler runs in SPI dispatch IRQ context. Empirical finding
 * (Phase 07.4.3 Path A handler-timing): when can_write fires from this
 * IRQ context, the SPI bridge's response-shadowing path stalls — adding
 * 87 ms latency to the SPI round-trip for SET_TARGETS while GET_DIAG
 * (no can_write) shadows cleanly at 1112 Hz.
 *
 * Fix: queue TX requests in a lock-free ring from the IRQ producer side;
 * drain (call can_write) from the H7 main loop (non-IRQ context). The
 * IRQ returns immediately, response shadows on the next SPI cycle, and
 * FDCAN TX happens out-of-band with the bridge.
 *
 * Ring size 32 — at 6 motors × ~4 register writes per LQR iter at 500 Hz
 * outer loop, peak rate is 12000 writes/sec. Main loop drains at >100 kHz
 * (each iteration does <10 µs of work per pending TX). Ring overflow at
 * 32 entries means main loop hasn't drained for >25 µs — should never
 * happen under healthy operation. Sized to fit within Phase 07.4 BSS
 * growth gate (5% over Phase 7.3 baseline). */
#define SMART_MD_TX_RING_SIZE 32u  /* must be power of 2 */
/* Phase 07.4.5 (patch 0040) — frame_kind discriminator for ring slots.
 * Distinguishes ordinary single-register WRITE_REGISTER_LEGACY frames
 * (SMART_MD_FRAME_WRITE_REG_SINGLE) from the multi-register Kp+Kd packed
 * frame (SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL) the F1 fix uses. */
#define SMART_MD_FRAME_WRITE_REG_SINGLE     0u
#define SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL 1u
typedef struct {
    uint16_t md_can_id;
    uint16_t reg_addr;
    uint8_t  value_size;
    uint8_t  frame_kind;   /* SMART_MD_FRAME_WRITE_* (Phase 07.4.5 patch 0040) */
    uint8_t  _pad[2];
    /* Phase 07.4.5 (patch 0040): widened from value[8] to value[16] to fit the
     * 12-byte multi-register WRITE_REGISTER_LEGACY payload (Kp+Kd) atomically
     * (F1 root-cause fix; CANdle-SDK MD.cpp:252-264 byte layout). */
    uint8_t  value[16];
} smart_md_pending_tx_t;
static smart_md_pending_tx_t s_pending_tx_ring[SMART_MD_TX_RING_SIZE];
static volatile uint32_t s_pending_tx_head = 0;  /* IRQ-context producer */
static volatile uint32_t s_pending_tx_tail = 0;  /* main-loop consumer */
static uint32_t s_pending_tx_overflow      = 0;  /* ring full count */

/* Phase 07.4.5 (patch 0040) — lockstep state machine constants + counters.
 *
 * BLOCKER-A: SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US is bench-driven from
 *   tests/phase07.4.5-lockstep-task0-bench.sh verdict.json. The 20260513T051524Z
 *   Task 0 run produced timeout_recommendation_us=500 (some opcodes' userspace-
 *   IOCTL round-trip exceeded 250 µs; H7-internal lockstep latency is expected
 *   to be ≤250 µs per RESEARCH §OQ-1 physical-layer math, but until Wave 2 sine-
 *   soak bench cache_age_p99_us validates that, the conservative 500 µs ceiling
 *   absorbs SPI bridge overhead in the per-iteration measurement). Wave 2 bench
 *   may shrink this to 250 µs if cache_age_p99 ≤ 1 ms across the tier sweep.
 *
 * SMART_MD_LOCKSTEP_OFFLINE_THRESHOLD: after 3 consecutive 500 µs timeouts on
 *   the same motor, mark it offline; periodic recheck heartbeat reactivates. */
#define SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US 500u  /* BLOCKER-A: Task 0 bench-driven */
#define SMART_MD_LOCKSTEP_OFFLINE_THRESHOLD   3u
static uint8_t  s_motor_offline_mask         = 0;
static uint8_t  s_motor_timeout_count[SMART_MD_MAX_MOTORS] = {0};
static uint32_t s_motor_recheck_counter      = 0;
static uint32_t s_lockstep_cycle_counter     = 0;
static uint32_t s_lockstep_tx_failures       = 0;
static uint32_t s_lockstep_responses_matched = 0;
static uint32_t s_lockstep_responses_timeout = 0;
static uint32_t s_handle_data_max_us         = 0;
static uint32_t s_liveness_tick              = 0;
static uint32_t s_md_writes_acked            = 0;
static uint32_t s_md_writes_no_ack           = 0;

/* Phase 07.4.3 Plan 03 Path A (patch 0033) — can_write timing instrumentation.
 * DWT->CYCCNT @ SYSCLK=400 MHz = DWT_CYCLES_PER_US (=400) cycles/µs. s_md_writes_total_us accumulates time
 * spent inside can_write across all emits; s_md_writes_max_us tracks slowest
 * single call. Isolates HAL_FDCAN_AddMessageToTxFifoQ latency. */
static uint32_t s_md_writes_total_us         = 0;
static uint32_t s_md_writes_max_us           = 0;

/* Bound for SET_TARGETS torque values — conservative literal; AK10-9 / AK60-6
 * hold-current envelope. T-07.4.3-01 STRIDE Tampering mitigation: clamp
 * |target_torque| <= MD80_TORQUE_MAX_SAFE before dispatching to MD80. Phase
 * 7.5 LQR zero-torque smoke test will not exceed this. */
#define MD80_TORQUE_MAX_SAFE 5.0f  /* Nm */

TIM_HandleTypeDef htim7_smart_md;

/*===========================================================================
 * CRC32 — IEEE 802.3 polynomial (0xEDB88320, reflected)
 *
 * Pure software implementation. Lightweight (table-less) since smart-MD frames
 * are ≤ 204 bytes (Phase 07.4.2 D1: 144 B request CRC range / 204 B response
 * CRC range; was 140 B for both at Phase 07.4) and CRC is computed once per
 * dispatch — under 6 µs on the M7 @ SYSCLK=400 MHz at the larger response size. If
 * profiling shows CRC dominating dispatch latency, swap for a lookup-table
 * version.
 *===========================================================================*/

static uint32_t smart_md_crc32(uint8_t const * data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint32_t b = 0; b < 8; ++b) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

/*===========================================================================
 * MD-protocol decoder — placeholder
 *
 * Wave 1 single-motor scope: motor 311 emits MD-protocol responses on FDCAN2.
 * The exact MD-protocol frame layout (position, velocity, torque, status word)
 * is documented in MOTOR_STACK_CONTEXT.md + CANdle-SDK headers; for Wave 1
 * the cache poll only validates that a frame from the known motor ID arrived,
 * and stamps s_state_cache[i].timestamp_us + cache_age_ms + md_can_id. The
 * fields position/velocity/torque are populated as a best-effort decode of
 * the MD-protocol mainState frame (12-byte payload at min). Multi-motor and
 * full MD-protocol decode (impedance_setpoint echo, fault flags) is reserved
 * for follow-on plan; Wave 1 demonstrates the transport, not the full motor
 * abstraction.
 *
 * The decode-from-MD-mainState mapping is intentionally minimal here. If
 * the bench in Task 6 reveals position/velocity values that don't track
 * physical motion, that's a Wave 2 polish item, not a Wave 1 gate failure
 * (Wave 1 gate is the realized A53 outer-loop rate, not data fidelity).
 *===========================================================================*/

static int smart_md_find_motor_index(uint16_t md_can_id)
{
  for (int i = 0; i < SMART_MD_KNOWN_MOTOR_COUNT; ++i) {
    if (s_known_md_can_ids[i] == md_can_id) {
      return i;
    }
  }
  return -1;
}

/*===========================================================================
 * Phase 07.4.3 Plan 02 Wave 1 — MD80 register-write frame emit helper
 *
 * Builds a CANdle-SDK MD80 WRITE_REGISTER_LEGACY frame and emits it on FDCAN2
 * (carrier-hat external CAN bus per Phase 06.3-06.5 hardware lock; motor IDs
 * 310..315 wired here per Phase 07.4.2 D1).
 *
 * Wire bytes (per CANdle-SDK MD.hpp:564-578 + md_types.hpp:153-163):
 *   buf[0] = 0x40   MdFrameId_E::WRITE_REGISTER_LEGACY (md_types.hpp:156)
 *   buf[1] = 0x00   pad (MD.hpp:566)
 *   buf[2..3]       reg_addr little-endian u16
 *   buf[4..]        value bytes (1 / 2 / 4 B per register width)
 * payload_len = 4 + value_size  (5 / 6 / 8 — all legal FDCAN-FD DLCs)
 *
 * Invariants:
 *   - Patch 0026 fdcan_2.State guard PRESERVED (carried into helper as
 *     equivalent early-return-with-counter-increment; cache_refresh_tick guard
 *     is unchanged at its own callsite).
 *   - X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS (BRS+FDF both set; per Pitfall 2
 *     and reference_md80_fleet_bitrate_5m_portenta — MD80s on Portenta X8 run
 *     1M arb / 5M data, requires BRS=ON).
 *   - fdcan_2 (NOT fdcan_1) — Pitfall 3, memory rule
 *     reference_portenta_x8_can_bus_mapping (FDCAN2 / can1 is the carrier-hat
 *     external CAN; FDCAN1 has no transceiver).
 *
 * Counter side-effects:
 *   s_md_writes_fdcan_not_ready++  if state guard fails (no emit)
 *   s_md_writes_tx_full++          if TX FIFO saturated (no emit)
 *   s_md_writes_emitted++          on successful can_write (rc == 0)
 *
 * Returns 0 on success, -1 otherwise. Per-motor failures are non-fatal at the
 * frame level — the dispatch caller still returns SMART_MD_STATUS_OK; the
 * operator validates motor activation via BATCH_READ_STATE.
 *===========================================================================*/

static int smart_md_emit_md_write(uint16_t md_can_id,
                                  uint16_t reg_addr,
                                  void const * value,
                                  uint8_t value_size)
{
  /* Phase 07.4.4 (patch 0034) — deferred-TX-dispatch. IRQ-side producer:
   * just enqueue the TX request into the ring. Main loop's drain function
   * (smart_md_dispatch_pending_writes) executes can_write from non-IRQ
   * context, allowing the SPI bridge to shadow the response without
   * stalling on FDCAN TX activity. */
  if (value_size > sizeof(s_pending_tx_ring[0].value)) {
    return -1;  /* payload too large for ring slot */
  }
  uint32_t head = s_pending_tx_head;
  uint32_t next_head = (head + 1u) & (SMART_MD_TX_RING_SIZE - 1u);
  if (next_head == s_pending_tx_tail) {
    s_pending_tx_overflow++;
    return -1;  /* ring full — main loop hasn't drained */
  }
  smart_md_pending_tx_t * slot = &s_pending_tx_ring[head];
  slot->md_can_id  = md_can_id;
  slot->reg_addr   = reg_addr;
  slot->value_size = value_size;
  if (value_size > 0u && value != (void const *)0) {
    memcpy(slot->value, value, value_size);
  }
  /* Phase 07.4.18 patch 0068: C2 producer-write marker (REG_SINGLE enqueue path).
   * slot->frame_kind is INTENTIONALLY NOT written here -- patch 0068 is forensic
   * instrumentation only. The fix landed in Phase 7.4.19 patch 0069 (see above).
   * Counter increment is BEFORE the existing barrier so the counter store is
   * ordered with the slot fields. */
  /* Phase 07.4.19 patch 0069: C2 fix -- explicit REG_SINGLE write before barrier.
   * Resolves the stale-DUAL slot-lifecycle bug observed in Phase 7.4.18 (consumer
   * read DUAL frame_kind at ~3x the rate producer wrote REG_SINGLE in both fresh
   * and warm sessions). The pre-patch implementation relied on BSS-zero or the
   * consumer-side reset to clear frame_kind; under no-ACK timeout the slot never
   * advances tail and frame_kind stays at whatever the prior tenant left. Explicit
   * write here makes the slot deterministic on every re-enqueue. */
  slot->frame_kind = SMART_MD_FRAME_WRITE_REG_SINGLE;  /* patch 0069: C2 fix */
  s_frame_kind_producer_writes++;
  __DMB();  /* publish slot contents before advancing head */
  s_pending_tx_head = next_head;
  /* NOTE: s_md_writes_emitted is incremented in the dispatch consumer
   * (smart_md_dispatch_pending_writes) AFTER can_write returns OK. */

  /* Phase 07.4.3 Plan 03 Path A (patch 0032) — RESPONSE-WAIT REMOVED.
   * Empirical finding (GET_DIAG counter-delta observation): the 50ms
   * HAL_GetTick busy-wait loop from patch 0029 caused H7 hardware reset
   * mid-bench. s_refresh_ticks_total dropped from ~33000 to ~4000 across
   * the bench, indicating BSS was zeroed by hw reset. Hypothesis: SysTick
   * is suppressed during smart_md_handler IRQ context → HAL_GetTick never
   * advances → loop spins → IWDG independent watchdog fires → reset.
   *
   * Reverting to fire-and-forget. If motor genuinely requires the response-
   * pair handshake (per CANdle-SDK MD.hpp:565-578), Phase 7.4.4 will
   * relocate the response-wait to non-IRQ context (deferred-TX-dispatch). */
  return 0;
}

/*===========================================================================
 * IRQ-context cache refresh tick — Phase 07.4.2 D2 PRIMARY (pipelined FDCAN drain)
 *
 * Called from HAL_TIM_PeriodElapsedCallback (TIM7) at 1 kHz. Drains FDCAN2
 * RX FIFO via can_read() — which wraps HAL_FDCAN_GetRxMessage, documented
 * interrupt-safe by ST HAL — decodes any frames whose ID matches a known
 * motor, and updates s_state_cache.
 *
 * Phase 07.4.2 D2 PRIMARY (pipelined RX-drain) bound: process up to
 * SMART_MD_MAX_MOTORS (6) frames per tick. Bounding the drain at the
 * fleet size (rather than the original 8) caps tick processing time to the
 * exact worst-case where every motor delivered one new frame in the prior
 * 1 ms window; excess frames remain in the FIFO until the next tick.
 * Per RESEARCH §3.2 the H7 acts as the RX-FIFO completion drain — TX-side
 * polling is upstream (motors auto-broadcast when enabled by the userspace
 * OphanimSystem path via the existing x8h7_can / CANdle-SDK substrate).
 *
 * Worst-case staleness for any single motor at 1 kHz tick: 6 × 1 ms = 6 ms
 * (one full pipeline pass — Wave 0 RESEARCH §3.2 LOCK). Average ≈ 3 ms.
 *
 * NEVER called from handle_data() in main.c — enforced by tests/phase07.4-smart-md-build.sh.
 *===========================================================================*/

/*===========================================================================
 * Phase 07.4.5 (patch 0040) — lockstep state machine
 *
 * Replaces the three-engine async pipeline (TIM7 cache_refresh_tick +
 * HAL_FDCAN_RxFifo0Callback intercept + deferred-TX-ring free-drain) with
 * a single deterministic lockstep loop driven from handle_data() in main.c.
 *
 * Per RESEARCH §3.2:
 *   1. Boot guard: skip if fdcan_2 not initialized.
 *   2. BO recovery preempt (carries forward Phase 7.4.6 v3 CCCR.INIT toggle).
 *   3. Idle: no active motors AND ring empty → return immediately.
 *   4. Interleave: every other cycle, drain one write before a poll.
 *      Otherwise, poll one motor (round-robin).
 *   5. After TX: DWT-spin SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US polling
 *      FDCAN2 RX FIFO0. On match: parse into s_state_cache (poll) or
 *      record ACK (write). On timeout: bump s_motor_timeout_count[i];
 *      mark offline at threshold.
 *
 * MAB-canonical: one frame in flight at all times. See CANdle-SDK
 * MD.hpp:692-702 transferCanFrame for the reference pattern.
 *===========================================================================*/

/* Phase 07.4.5: DWT cycle counter (already enabled in smart_md_handler_init at
 * lines 1196-1202 via CoreDebug->DEMCR.TRCENA + DWT->CYCCNTENA) gives sub-µs
 * timeout precision. CM7 @ 480 MHz → 480 cycles per µs. */
static inline bool dwt_rx_match_or_timeout(uint32_t md_id,
                                           uint16_t reg_addr,
                                           uint32_t timeout_us,
                                           uint8_t *out_data,
                                           uint8_t *out_len)
{
  const uint32_t timeout_cycles = timeout_us * DWT_CYCLES_PER_US;
  const uint32_t t_start        = DWT->CYCCNT;
  while ((DWT->CYCCNT - t_start) < timeout_cycles) {
    if (can_rx_fifo_available(&fdcan_2, FDCAN_RX_FIFO0) == 0) {
      continue;
    }
    uint32_t rx_can_id = 0;
    uint8_t  rx_len    = 0;
    uint8_t  rx_flags  = 0;
    uint8_t  rx_buf[64] = {0};
    if (can_read(&fdcan_2, &rx_can_id, &rx_len, rx_buf, &rx_flags) == 0) {
      continue;
    }
    if (rx_can_id != md_id) {
      continue;
    }
    if (rx_len < 2u) {
      continue;
    }
    /* Phase 07.4.15 patch 0060 — P1 forensic counter site.
     * Increments BEFORE matcher's frame_kind + reg_addr filters so we count
     * what the H7 actually RECEIVES regardless of acceptance. */
    switch (rx_buf[0]) {
      case 0xA0: s_md_rx_first_byte_A0++;    break;
      case 0x41: s_md_rx_first_byte_41++;    break;
      case 0x42: s_md_rx_first_byte_42++;    break;
      default:   s_md_rx_first_byte_other++; break;
    }
    if (rx_len >= 4u) {
      switch (rx_buf[2]) {
        case 0x50: s_md_rx_reg_addr_lo_50++;    break;
        case 0x51: s_md_rx_reg_addr_lo_51++;    break;
        case 0x62: s_md_rx_reg_addr_lo_62++;    break;
        case 0x63: s_md_rx_reg_addr_lo_63++;    break;
        case 0x64: s_md_rx_reg_addr_lo_64++;    break;
        case 0x40: s_md_rx_reg_addr_lo_40++;    break;
        case 0x41: s_md_rx_reg_addr_lo_41++;    break;
        case 0x42: s_md_rx_reg_addr_lo_42++;    break;
        case 0x00: s_md_rx_reg_addr_lo_00++;    break;
        default:   s_md_rx_reg_addr_lo_other++; break;
      }
    }
    /* Ring-buffer write (16-slot, 8-byte snapshots). */
    {
      uint8_t slot_idx = s_md_rx_ring_head & 0xFu;
      uint8_t copy_len = (rx_len < 8u) ? (uint8_t)rx_len : 8u;
      for (uint8_t i = 0; i < copy_len; ++i) {
        s_md_rx_ring[slot_idx][i] = rx_buf[i];
      }
      for (uint8_t i = copy_len; i < 8u; ++i) {
        s_md_rx_ring[slot_idx][i] = 0u;
      }
      s_md_rx_ring_head = (uint8_t)((slot_idx + 1u) & 0xFu);
      s_md_rx_ring_total++;
    }
    /* Accept RESPONSE_LEGACY (0xA0) or echo (0x41 / 0x42). reg_addr==0 means
     * "any address" (caller doesn't care — used by write-ACK matching). */
    if (rx_buf[0] != 0xA0 && rx_buf[0] != 0x41 && rx_buf[0] != 0x42) {
      continue;
    }
    /* Phase 07.4.15 Plan 02 patch 0063 — H-B3 wire-format amendment.
     *
     * EMPIRICAL ROOT CAUSE (Task 2 forensic re-run, runs/phase07.4.15-forensic/20260516T234536Z/):
     *   Across 203,194 received RESPONSE_LEGACY (0xA0) frames:
     *     - byte[0] = 0xA0 in 99.999% of frames
     *     - byte[2] = 0x00 in 99.999% of frames (UNIFORMLY 0x00)
     *     - Counters for byte[2]==0x50/0x51/0x40/0x41/0x42/0x62/0x63/0x64 all read 0
     *     - Ring buffer dump: 16/16 slots match A0 00 00 00 | <payload>
     *
     *   RESPONSE_LEGACY (0xA0) frames from MAB Robotics MD80 motors do NOT
     *   echo the written reg_addr in bytes [2:4]; bytes [2:4] are reserved
     *   /status (byte[2] = 0x00 = OK status). Patch 0058's A3 matcher
     *   tightening (which compared resp_addr := rx_buf[2..3] to expected
     *   reg_addr) therefore rejected every legitimate WRITE-ACK frame.
     *   Counter evidence: s_write_ack_reg_addr_mismatch = 203,194 while
     *   s_md_writes_acked = 0 (0% match rate).
     *
     *   The 0x41 / 0x42 echo frames (READ_REGISTER and other echo paths)
     *   MAY still encode reg_addr in byte[2..3] per CANdle convention; the
     *   reg_addr check is preserved for those paths.
     *
     * FIX:
     *   Condition the reg_addr equality check on rx_buf[0] != 0xA0 — so
     *   RESPONSE_LEGACY frames bypass the check (matcher accepts on
     *   frame_kind + can_id alone, consistent with the empirical wire
     *   format observed in the field). 0x41 / 0x42 frames continue to
     *   exercise the check unchanged.
     *
     * FALSIFIERS (post-fix expected):
     *   s_md_writes_acked should approach s_md_writes_emitted (ratio >= 0.95)
     *   s_write_ack_reg_addr_mismatch should remain at most the count of
     *     non-0xA0 frames with reg_addr mismatch (low; near zero in steady state).
     */
    if (reg_addr != 0u && rx_len >= 4u && rx_buf[0] != 0xA0) {
      uint16_t resp_addr = (uint16_t)rx_buf[2] | ((uint16_t)rx_buf[3] << 8);
      if (resp_addr != reg_addr) {
        /* Phase 07.4.14 patch 0058 (A3): falsifier — counts mismatched-reg
         * responses rejected by the tightened matcher. Pre-fix the WRITE-ACK
         * call site passed 0u so this branch was unreachable from WRITE-ACK.
         * Phase 07.4.15 patch 0063: now only counts 0x41/0x42 echo-frame
         * mismatches; 0xA0 RESPONSE_LEGACY frames bypass this gate. */
        s_write_ack_reg_addr_mismatch++;
        continue;
      }
    } else if (reg_addr != 0u && rx_buf[0] == 0xA0) {
      /* Phase 07.4.20 patch 0071: 0xA0 bypass fired -- patch 0063 fix IS active.
       * Increment s_write_ack_0xa0_bypassed to prove bypass is compiled into this ELF.
       * This counter growing during activate is the primary Strategy E falsifier:
       * motors return 0xA0 ACKs AND the H7 correctly bypasses the reg_addr check,
       * allowing s_md_writes_acked to advance past the mismatch-retry loop. */
      s_write_ack_0xa0_bypassed++;
      /* Phase 07.4.21 patch 0072 (D-04) -- falsifier for F-02 (belt-and-suspenders
       * on Plan 01's static-evidence finding). Confirms WRITE-ACK 0xA0 payload
       * carries non-zero bytes beyond the 4-byte header in the deployed ELF. */
      if (rx_len > 4u) {
        for (uint8_t i = 4u; i < rx_len && i < 16u; ++i) {
          if (rx_buf[i] != 0u) { s_writeack_payload_nonzero_count++; break; }
        }
      }
      /* PHASE_0073_H4_DISAMBIG: motor fault bitmask diagnostic.
       * MAB authoritative 0xA0 schema: bytes [1..2] = QUICK STATUS u16 fault
       * bitmask. Byte [3] = MOTOR TEMPERATURE (excluded -- always non-zero on
       * running motors). RESEARCH Open Q3 RESOLVED INCLUDE: surfaces motor
       * fault states without GET_DIAG poll. */
      if (rx_len >= 3u && (rx_buf[1] != 0u || rx_buf[2] != 0u)) {
        s_writeack_quick_status_nonzero_count++;
      }
    }
    if (out_data != (void *)0) memcpy(out_data, rx_buf, rx_len);
    if (out_len  != (void *)0) *out_len = rx_len;
    return true;
  }
  /* PHASE_0073_H4_DISAMBIG: retry-path accumulator. Gated by
   * s_in_set_targets_dispatch (set/cleared at SET_TARGETS entry/exit) so
   * only SET_TARGETS-originated timeouts increment. Single increment per
   * timeout (not per matcher loop iteration) -- maps to "SET_TARGETS
   * WRITE-ACK needed a retry" semantics for Plan 03 criterion-7. */
  if (s_in_set_targets_dispatch != 0u) {
    s_set_targets_handshake_retry_count++;
  }
  return false;
}

/* Parse a 3-register READ_REGISTER bulk response payload into s_state_cache. */
static void smart_md_parse_response_into_cache(int idx, const uint8_t *data, uint8_t len)
{
  if (idx < 0) return;
  if (len < (2u + SMART_MD_POLL_REG_COUNT * (2u + 4u))) return;
  motor_state_t * cache = &s_state_cache[idx];
  for (uint32_t i = 0; i < SMART_MD_POLL_REG_COUNT; ++i) {
    const uint32_t off = 2u + i * 6u;
    uint16_t addr = (uint16_t)data[off] | ((uint16_t)data[off + 1] << 8);
    float    val;
    memcpy(&val, &data[off + 2u], sizeof(float));
    if      (addr == 0x062) cache->velocity = val;
    else if (addr == 0x063) cache->position = val;
    else if (addr == 0x064) cache->torque   = val;
  }
  uint32_t now_ms = HAL_GetTick();
  cache->timestamp_us = now_ms * 1000u;
  cache->md_can_id    = s_known_md_can_ids[idx];
  cache->cache_age_ms = 0;
}

/* Advance round-robin to next active, non-offline motor. Returns -1 if none. */
static int smart_md_next_pollable_motor(void)
{
  for (int n = 0; n < SMART_MD_KNOWN_MOTOR_COUNT; ++n) {
    uint8_t idx = (uint8_t)((s_poll_motor_idx + (uint8_t)n) % SMART_MD_KNOWN_MOTOR_COUNT);
    uint8_t bit = (uint8_t)(1u << idx);
    if ((s_poll_active_mask & bit) == 0u) continue;
    /* Phase 07.4.9 patch 0047 — OR absent_mask into the skip predicate.
     * absent_mask is seeded at BUS_PROBING→BUS_HEALTHY entry from the
     * responders bitmask; closes cause-3 NACK-storm on absent slots. */
    if (((s_motor_offline_mask | s_motor_absent_mask) & bit) != 0u) continue;
    return (int)idx;
  }
  return -1;
}

/* TX_POLL → WAIT_RESP → PARSE_RESP for ONE round-robin motor. */
static void smart_md_do_poll_lockstep(void)
{
  int motor_idx = smart_md_next_pollable_motor();
  if (motor_idx < 0) return;
  uint16_t md_can_id = s_known_md_can_ids[motor_idx];

  /* Build READ_REGISTER 3-reg bulk frame (existing Phase 7.4.6 layout). */
  uint8_t tx_buf[2u + SMART_MD_POLL_REG_COUNT * (2u + 4u)] = {0};
  tx_buf[0] = 0x41;  /* READ_REGISTER (MdFrameId_E::READ_REGISTER) */
  tx_buf[1] = 0x00;
  for (uint32_t i = 0; i < SMART_MD_POLL_REG_COUNT; ++i) {
    const uint32_t off = 2u + i * 6u;
    tx_buf[off + 0] = (uint8_t)(s_poll_reg_addrs[i] & 0xFFu);
    tx_buf[off + 1] = (uint8_t)((s_poll_reg_addrs[i] >> 8) & 0xFFu);
  }

  if (can_tx_fifo_available(&fdcan_2) == 0u) {
    s_lockstep_tx_failures++;
    return;
  }
  int rc = can_write(&fdcan_2, (uint32_t)md_can_id, sizeof(tx_buf), tx_buf,
                     X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS);
  if (rc != 0) {
    s_lockstep_tx_failures++;
    return;
  }
  s_poll_emitted_total++;

  uint8_t rx_buf[64] = {0};
  uint8_t rx_len     = 0;
  /* reg_addr argument is the FIRST reg in the bulk (0x062 velocity) — the
   * MD80 echoes the register-block in the order we asked. */
  if (dwt_rx_match_or_timeout((uint32_t)md_can_id, s_poll_reg_addrs[0],
                              SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US,
                              rx_buf, &rx_len)) {
    smart_md_parse_response_into_cache(motor_idx, rx_buf, rx_len);
    s_motor_timeout_count[motor_idx] = 0;
    s_motor_offline_mask &= (uint8_t)~(1u << motor_idx);
    s_lockstep_responses_matched++;
    s_poll_responses_drained++;
    /* Phase 07.4.7 patch 0042: response-gated round-robin advance.
     * Only advance the cursor on a SUCCESSFUL drain. Restores the "one
     * frame in flight" invariant unintentionally violated by Phase 7.4.6
     * patch 0042 (reverted 2d5f545). See 07.4.7-01-AUDIT.md sec 4. */
    s_poll_motor_idx = (uint8_t)((motor_idx + 1) % SMART_MD_KNOWN_MOTOR_COUNT);
  } else {
    s_motor_timeout_count[motor_idx]++;
    if (s_motor_timeout_count[motor_idx] >= SMART_MD_LOCKSTEP_OFFLINE_THRESHOLD) {
      s_motor_offline_mask |= (uint8_t)(1u << motor_idx);
    }
    s_lockstep_responses_timeout++;
    /* Phase 07.4.7 patch 0042: NO advance on timeout — re-poll same motor
     * next cycle. After SMART_MD_LOCKSTEP_OFFLINE_THRESHOLD = 3 consecutive
     * timeouts the offline bit sets and smart_md_next_pollable_motor()
     * skips this motor; patch 0041's 1 Hz TIM7 recheck clears the mask. */
  }
}

/* TX_WRITE → WAIT_ACK → PARSE_ACK from ring tail. */
static void smart_md_do_write_lockstep(void)
{
  if (s_pending_tx_tail == s_pending_tx_head) return;
  __DMB();
  smart_md_pending_tx_t * slot = &s_pending_tx_ring[s_pending_tx_tail];
  /* Phase 07.4.18 patch 0068: C2 consumer-read marker + torn-read detection +
   * dedicated stale-DUAL signal (RESEARCH OQ-2). The differential (fresh vs warm)
   * of s_frame_kind_consumer_dual_reads vs s_md_writes_frame_kind_one (existing
   * P2 DUAL producer counter, line 259) is the primary C2 race signal per
   * RESEARCH Pitfall 2. torn_read_count > 0 is belt-and-suspenders only --
   * uint8_t LDRB is byte-atomic on Cortex-M7 (RESEARCH Finding 8 / A1). */
  {
    uint8_t fk = slot->frame_kind;
    s_frame_kind_consumer_reads++;
    if (fk > 1u) {
      s_frame_kind_torn_read_count++;
    } else if (fk == SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL) {
      s_frame_kind_consumer_dual_reads++;
    }
  }

  uint8_t buf[16] = {0};
  uint8_t payload_len = 0;

  if (slot->frame_kind == SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL) {
    s_md_writes_frame_kind_one++;  /* Phase 07.4.11 patch 0049 (D-02): G13 verification counter */
    /* Phase 07.4.10 patch 0048: insert mandatory padding byte after the
     * 0x40 WRITE_REGISTER_LEGACY opcode to match CANdle-SDK MD.hpp:565-567
     * writeRegisters byte emitter. Phase 7.4.5 F1 fix referenced
     * MD.cpp:252-264 (the setImpedanceParams CALLER) and missed the
     * padding — emitted 13 bytes instead of 14. Motor silently accepted
     * the malformed frame on the wire (status=OK echo) but refused to
     * update IMPEDANCE Kp/Kd RAM, falling back to FLASH gains and
     * masquerading as a motor-firmware RAM-vs-FLASH quirk. */
    buf[0] = 0x40;
    buf[1] = 0x00;                              /* mandatory padding per MD.hpp:567 */
    /* slot->value is the 12-byte packed payload:
     *   [0x50, 0x00, kp_LE_4B, 0x51, 0x00, kd_LE_4B] */
    if (slot->value_size > 0u) {
      uint8_t n = slot->value_size;
      if (n > 14u) n = 14u;                     /* buf[2..15] capacity = 14 (was 15) */
      memcpy(&buf[2], slot->value, n);          /* shifted from &buf[1] to make room for padding */
      payload_len = (uint8_t)(2u + n);          /* opcode + padding + n value bytes */
    } else {
      payload_len = 2u;                         /* opcode + padding only */
    }
  } else {
    /* SMART_MD_FRAME_WRITE_REG_SINGLE (default — patch 0034 byte layout). */
    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = (uint8_t)(slot->reg_addr & 0xFFu);
    buf[3] = (uint8_t)((slot->reg_addr >> 8) & 0xFFu);
    if (slot->value_size > 0u) {
      uint8_t n = slot->value_size;
      if (n > 12u) n = 12u;
      memcpy(&buf[4], slot->value, n);
      payload_len = (uint8_t)(4u + n);
    } else {
      payload_len = 4u;
    }
  }

  if (fdcan_2.Instance == (void *)0) {
    s_md_writes_fdcan_not_ready++;
    /* Phase 07.4.20 patch 0070: Layer B -- FDCAN2 not initialised. */
    s_fdcan2_tx_attempt_count++;
    s_fdcan2_tx_fail_busoff_count++;
  } else if (can_tx_fifo_available(&fdcan_2) == 0u) {
    s_md_writes_tx_full++;
    /* Phase 07.4.20 patch 0070: Layer B -- TX FIFO full. */
    s_fdcan2_tx_attempt_count++;
    s_fdcan2_tx_fail_full_count++;
  } else {
    /* Phase 07.4.20 patch 0070: Layer B -- FDCAN2 TX FIFO instrumentation.
     * Pre-call: check bus state and log attempt. One-shot: if bus is not HEALTHY
     * at dispatch and this is the first emission attempt (s_md_writes_emitted == 0),
     * capture bus state value for diagnosis (0=RESETTING, 1=PROBING, 3=OFF, 4=LINK_DOWN). */
    if (s_bus_state != BUS_HEALTHY && s_fdcan2_tx_busy_at_activate == 0u
        && s_md_writes_emitted == 0u) {
      s_fdcan2_tx_busy_at_activate = (uint32_t)s_bus_state;  /* encode state value */
    }
    if (s_bus_state != BUS_HEALTHY) {
      s_fdcan2_tx_fail_busoff_count++;
    } else {
      s_fdcan2_tx_attempt_count++;
    }
    int rc = can_write(&fdcan_2, (uint32_t)slot->md_can_id, payload_len, buf,
                       X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS);
    if (rc == 0) {
      s_fdcan2_tx_success_count++;
      s_md_writes_emitted++;
      /* Wait for WRITE-ACK (0xA0 RESPONSE_LEGACY or 0x42 WRITE_REGISTER echo).
       * Phase 07.4.14 patch 0058 (A3): pass slot->reg_addr (not 0u) so the
       * matcher requires resp_addr correctness. DUAL slot sets reg_addr=0x050
       * at smart_md_handler.c:1474; single-reg writes set reg_addr to the
       * emitted register address. RESEARCH §5. */
      if (dwt_rx_match_or_timeout((uint32_t)slot->md_can_id, slot->reg_addr,
                                  SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US,
                                  (void *)0, (void *)0)) {
        s_md_writes_acked++;
        /* Phase 07.4.7 patch 0042: response-gated round-robin advance for
         * the write path — advance the pending-TX-ring tail only on
         * successful WRITE-ACK. Timed-out writes are re-driven next call. */
        __DMB();
        /* Phase 07.4.11 patch 0049 (D-01): consumer-side reset of slot->frame_kind
         * to clear stale DUAL marker before this slot is reused by a future
         * smart_md_emit_md_write call. Prevents the patch-0048 era ring-reuse bug
         * where slots that previously held SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL
         * caused subsequent single-register writes to serialize via the DUAL
         * builder, emitting malformed 6-byte frames the motor silently rejected.
         * Codex REVIEW.md Finding 1 root cause. Consumer-side location chosen
         * over producer-side (patch-0049-producer-side, reverted at 9f8ac8b)
         * to test whether the regression cause is producer-side-specific. */
        /* Phase 07.4.19 patch 0069: pool sentinel (was REG_SINGLE). Consumer marks
         * slot returned-to-pool with 0xFFu; producer's next smart_md_emit_md_write
         * call (re-enqueue) MUST overwrite with REG_SINGLE explicitly. If the
         * consumer sees 0xFFu in a future dispatch (slot reuse before producer
         * commitment), s_frame_kind_torn_read_count increments (was BSS-corruption
         * detector; now also a producer-commitment-race detector per Pitfall 5). */
        s_pending_tx_ring[s_pending_tx_tail].frame_kind = 0xFFu;
        s_pending_tx_tail = (s_pending_tx_tail + 1u) & (SMART_MD_TX_RING_SIZE - 1u);
      } else {
        s_md_writes_no_ack++;
        /* Phase 07.4.12 patch 0050 (D-03): CF-1 falsifier — track no-ACK
         * timeouts that fired on a DUAL frame. If this > 0 after the
         * activate sequence, the no-ACK stall branch is leaving DUAL
         * frame_kind poisoned in the ring slot AND patch 0049's
         * consumer-side reset (which fires only on the ACK-success
         * branch) is insufficient. Phase 7.4.13 D-06 fix candidate:
         * extend consumer-side reset to the no-ACK branch. */
        if (slot->frame_kind == SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL) {
          s_md_writes_no_ack_at_frame_kind_one++;
        }
        /* Phase 07.4.7 patch 0042: NO tail advance on WRITE timeout —
         * re-emit the same pending-write on the next call. */
      }
    } else {
      /* TX failed (can_write returned non-zero) — leave tail; next call retries. */
    }
  }
}

/*===========================================================================
 * Phase 07.4.8 patch 0044 — Layer 2 top-level H7 bus state machine
 *
 * Implements the 5-state machine BUS_RESETTING → BUS_PROBING → BUS_HEALTHY
 * → BUS_OFF → BUS_LINK_DOWN per CONTEXT D-02/D-03 + RESEARCH Patterns 1-4.
 *
 * Ticked once per handle_data() iteration from main.c BEFORE can_handle_data()
 * (so the Layer 1 polling gate sees fresh state) and BEFORE smart_md_lockstep_step
 * (so the BUS_HEALTHY guard reads fresh state).
 *
 * BUS_RESETTING entry implements Bosch M_CAN AN004 §2 BO recovery:
 *   1. Clear FDCAN_IR_BO|EP|EW + flush TX ring (D-04a — without this, retrans
 *      storm immediately re-drives BO post-recovery; Pitfall 2).
 *   2. Set CCCR.INIT (idempotent — already latched on cold-flash BO=1).
 *   3. Clear CCCR.INIT.
 *   4. tick_in_RESETTING() polls PSR.ACT == 1 (NOT CCCR.INIT == 0 — Pitfall 1,
 *      Codex REVIEW #2: AN004 §2 mandates 129 occurrences of Bus Idle =
 *      11 consecutive recessive bits; completion is PSR.ACT==1).
 *   5. 20 ms guard timer falls through to BUS_OFF (exp backoff).
 *
 * Negative invariants (smart_md_handler.c:35-37): NO HAL_FDCAN_Init/Start/
 * Stop/DeInit. All FDCAN register access via raw fdcan_2.Instance->CCCR /
 * IR / PSR / ECR reads/writes (same pattern as deleted in-lockstep preempt).
 *===========================================================================*/

static const char *bus_state_name(enum smart_md_bus_state s)
{
  switch (s) {
    case BUS_RESETTING:  return "BUS_RESETTING";
    case BUS_PROBING:    return "BUS_PROBING";
    case BUS_HEALTHY:    return "BUS_HEALTHY";
    case BUS_OFF:        return "BUS_OFF";
    case BUS_LINK_DOWN:  return "BUS_LINK_DOWN";
    default:             return "BUS_?";
  }
}

static void bus_state_transition(enum smart_md_bus_state next)
{
  s_bus_state          = next;
  s_bus_state_enter_ms = HAL_GetTick();
  if (s_bus_state_transitions[next] < 255u) {
    s_bus_state_transitions[next]++;
  }
  if (next == BUS_OFF) {
    s_bo_off_enter_ms = HAL_GetTick();
  }
  if (next == BUS_RESETTING) {
    /* Reset BUS_PROBING per-pass state (next entry into PROBING reseeds). */
    s_bus_probe_motor_idx = 0u;
    s_bus_probe_responses = 0u;
    /* Phase 07.4.9 patch 0047 — clear absent_mask + responders bitmask on
     * BUS_RESETTING entry so the lockstep cannot see stale absent bits
     * while the bus is rebuilding state (D-03a). */
    s_motor_absent_mask            = 0u;
    s_bus_probe_responders_bitmask = 0u;
  }
  if (next == BUS_PROBING) {
    /* Phase 07.4.9 patch 0047 — clear responders bitmask on BUS_PROBING
     * entry (mirrors the existing s_bus_probe_responses = 0u sibling
     * reset above; D-03b). */
    s_bus_probe_responders_bitmask = 0u;
  }
  (void)bus_state_name;  /* reserved for future #if DEBUG_H7_BUS_STATE_TRACE pr_info */
}

/* RESEARCH Pattern 1 verbatim: AN004 §2 conformant BO recovery entry.
 * Clears latched error IRs, flushes TX ring (D-04a), sets+clears CCCR.INIT.
 * The PSR.ACT==1 wait happens in bus_state_tick_in_RESETTING() (non-blocking,
 * polled across main-loop iterations with a 20 ms guard timer). */
static void bus_enter_RESETTING(void)
{
  if (fdcan_2.Instance == (void *)0) {
    bus_state_transition(BUS_RESETTING);
    return;
  }

  /* (a) Clear latched error IR bits (write-1-to-clear). */
  fdcan_2.Instance->IR = (FDCAN_IR_BO | FDCAN_IR_EP | FDCAN_IR_EW);

  /* (b) D-04a: TX-ring flush BEFORE CCCR.INIT toggle. Without this, queued
   * writes immediately re-drive BO post-recovery (Pitfall 2). The producer
   * is the SPI dispatch path (IRQ context); the consumer is this main-loop
   * tick. Synchronise tail with head; __DMB() ensures the store is visible
   * before subsequent CCCR pokes. */
  s_pending_tx_tail = s_pending_tx_head;
  __DMB();

  /* (c) Set CCCR.INIT (idempotent — already latched on cold-flash BO=1). */
  fdcan_2.Instance->CCCR |= FDCAN_CCCR_INIT;

  /* (d) Clear CCCR.INIT immediately. AN004 §2 wait happens in tick_in_RESETTING
   * by polling PSR.ACT==1 — NOT by spinning on CCCR.INIT (Pitfall 1). */
  fdcan_2.Instance->CCCR &= ~FDCAN_CCCR_INIT;

  /* G3 verification preservation: bus_enter_RESETTING is the new BO-recovery
   * counter increment site (the deleted in-lockstep preempt was the previous one). */
  s_bo_recovery_attempts++;
  s_bo_recovery_last_ms = HAL_GetTick();

  bus_state_transition(BUS_RESETTING);
}

/* RESEARCH Pattern 1 verbatim: poll PSR.ACT==1 (NOT CCCR.INIT==0) with a 20 ms
 * guard timer. AN004 §2: M_CAN waits 129 × Bus Idle (11 consecutive recessive
 * bits each) = ~1.4 ms at 1 Mbit/s arbitration phase before resuming; completion
 * signal is PSR.ACT == 1, PSR.BO clear, ECR.TEC/REC clear. */
static void bus_state_tick_in_RESETTING(void)
{
  if (fdcan_2.Instance == (void *)0) return;

  uint32_t psr = fdcan_2.Instance->PSR;
  uint32_t act = (psr & FDCAN_PSR_ACT_Msk) >> FDCAN_PSR_ACT_Pos;
  /* AN004 completion: ACT == 0b01 (Idle) AND BO clear. */
  if (act == 1u && (psr & FDCAN_PSR_BO) == 0u) {
    bus_state_transition(BUS_PROBING);
    return;
  }

  /* 20 ms guard timer falls through to BUS_OFF (exp backoff path). */
  if ((HAL_GetTick() - s_bus_state_enter_ms) >= SMART_MD_BUS_RESETTING_GUARD_MS) {
    bus_state_transition(BUS_OFF);
  }
}

/* RESEARCH Pattern 2 verbatim: BUS_PROBING — one READ_REGISTER per known MD ID,
 * 5 ms DWT timeout each. After full sweep, ≥ 1 response → BUS_HEALTHY; zero
 * responses → BUS_LINK_DOWN (terminal). Preserves the "one frame in flight"
 * invariant — single TX, wait, then next. */
static void bus_state_tick_in_PROBING(void)
{
  if (fdcan_2.Instance == (void *)0) return;
  if (can_tx_fifo_available(&fdcan_2) == 0u) return;

  uint16_t md_can_id = s_known_md_can_ids[s_bus_probe_motor_idx];
  /* Build a single-register READ_REGISTER frame: reg 0x000 (firmwareVersion u32). */
  uint8_t buf[8] = { 0x41u, 0x00u, 0x00u, 0x00u, 0u, 0u, 0u, 0u };
  uint8_t rx_buf[64] = {0};
  uint8_t rx_len = 0;

  int rc = can_write(&fdcan_2, (uint32_t)md_can_id, sizeof(buf), buf,
                     X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS);
  if (rc == 0) {
    if (dwt_rx_match_or_timeout((uint32_t)md_can_id, 0x0000u,
                                SMART_MD_BUS_PROBING_TIMEOUT_US,
                                rx_buf, &rx_len)) {
      s_bus_probe_responses++;
      /* Phase 07.4.9 patch 0047: per-pass responder bitmask. Bit set
       * here identifies WHICH motor responded (the counter only knows
       * HOW MANY). Complement-masked at BUS_HEALTHY entry (D-02). */
      s_bus_probe_responders_bitmask |= (uint8_t)(1u << s_bus_probe_motor_idx);
      /* Phase 07.4.8 patch 0046: lifetime saturating probe response
       * counter (instrumentation only — no behavior change). */
      if (s_bus_probe_responses_total < 255u) {
        s_bus_probe_responses_total++;
      }
    }
  }

  s_bus_probe_motor_idx++;
  if (s_bus_probe_motor_idx >= SMART_MD_KNOWN_MOTOR_COUNT) {
    if (s_bus_probe_responses >= 1u) {
      /* Phase 07.4.9 patch 0047 — seed absent_mask from PROBING responders.
       * Bits set in absent_mask = slots that did NOT respond to the probe
       * sweep; the lockstep skip predicate ORs this into s_motor_offline_mask
       * to drop those slots from the round-robin. Closes cause-3 NACK-storm
       * per 07.4.8 T1-diagnosis.md (D-03). */
      s_motor_absent_mask = (uint8_t)(~s_bus_probe_responders_bitmask)
                            & SMART_MD_KNOWN_MOTOR_VALID_MASK;
      bus_state_transition(BUS_HEALTHY);
      /* Patch 0042 (response-gated lockstep advance) now runs unchanged in
       * smart_md_lockstep_step — gated by the BUS_HEALTHY guard. */
    } else {
      /* Phase 07.4.8 patch 0046: capture BO-trigger reason at the only
       * BUS_PROBING -> BUS_LINK_DOWN transition site. Reason = sweep
       * completed with zero MD80 responses. Candidate-cause-2 signal
       * per CONTEXT.md instrumentation_objective + T1-decoded-state.json. */
      s_last_bo_trigger_reason = BO_REASON_PROBING_ZERO_RESPONSES;
      bus_state_transition(BUS_LINK_DOWN);
    }
  }
}

/* RESEARCH Pattern 4 verbatim: exponential backoff 10 → 50 → 250 → 1000 ms cap
 * per CONTEXT D-02d. After hitting the 1 s cap, also honour the "quiet 5 s"
 * predicate (D-02d) — if no Linux writes during the 5 s window, allow re-attempt;
 * otherwise stay in BUS_OFF to prevent recovery storm under permanent fault. */
static void bus_state_tick_in_OFF(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms = now_ms - s_bus_state_enter_ms;

  if (elapsed_ms < s_bo_backoff_ms) {
    return;
  }

  /* Backoff elapsed — re-attempt BUS_RESETTING. Grow backoff for next entry. */
  uint32_t next = s_bo_backoff_ms * SMART_MD_BO_BACKOFF_GROWTH;
  if (next > SMART_MD_BO_BACKOFF_MAX_MS) next = SMART_MD_BO_BACKOFF_MAX_MS;
  s_bo_backoff_ms = next;

  /* D-02d "quiet 5 s" predicate: if backoff is already at the cap AND there
   * have been Linux writes during the 5 s window (s_md_writes_emitted delta),
   * stay in BUS_OFF (bus wiring fault likely). Otherwise enter BUS_RESETTING. */
  if (s_bo_backoff_ms >= SMART_MD_BO_BACKOFF_MAX_MS) {
    if ((now_ms - s_bo_off_enter_ms) < SMART_MD_BO_QUIET_REATTEMPT_MS) {
      /* Within first 5 s of BUS_OFF — stay until quiet window elapses OR
       * operator reset. The backoff cap holds; we just defer re-entry. */
      s_bus_state_enter_ms = now_ms;  /* arm the next backoff tick */
      return;
    }
  }

  bus_enter_RESETTING();
}

void smart_md_bus_state_tick(void)
{
  if (fdcan_2.Instance == (void *)0) {
    return;
  }

  /* Phase 07.4.15 Plan 02 patch 0062 — cold-boot bus state machine init fix.
   * On the FIRST tick after fdcan_2.Instance becomes valid (i.e. AFTER
   * on_CAN_INIT_Request fired for FDCAN2 and HAL_FDCAN_Start ran), force
   * a single bus_enter_RESETTING() invocation. The static initializer
   * `s_bus_state = BUS_RESETTING` skips the entry routine — this one-shot
   * compensates so CCCR.INIT is explicitly toggled, the TX ring is flushed,
   * s_bus_state_transitions[BUS_RESETTING] increments (proving the entry
   * routine ran), and s_bus_state_enter_ms is re-armed to a known epoch
   * for the 20 ms guard timer in tick_in_RESETTING().
   *
   * Runs exactly once per H7 boot (s_cold_init_done is BSS-init false; set
   * here on first call). Subsequent recoveries follow the unchanged path
   * (BUS_OFF -> bus_state_tick_in_OFF -> bus_enter_RESETTING). */
  if (!s_cold_init_done && s_bus_state == BUS_RESETTING) {
    s_cold_init_done = true;
    s_cold_boot_init_fix_invocations++;
    bus_enter_RESETTING();
    /* bus_enter_RESETTING() already invoked bus_state_transition(BUS_RESETTING)
     * which re-set s_bus_state to BUS_RESETTING and incremented the transition
     * counter. Fall through to the per-state dispatch so the first tick still
     * polls PSR.ACT for the BUS_PROBING transition. */
  }

  /* Watchdog: BO latching from any non-OFF/RESETTING state forces BUS_OFF.
   * Run BEFORE the per-state dispatch so BUS_HEALTHY → BUS_OFF transition
   * happens before the next smart_md_lockstep_step (RESEARCH Pattern 3). */
  if ((fdcan_2.Instance->PSR & FDCAN_PSR_BO) != 0u &&
      s_bus_state != BUS_OFF && s_bus_state != BUS_RESETTING) {
    /* Phase 07.4.8 patch 0046: capture BO-trigger reason at the watchdog
     * site (the only BUS_HEALTHY/PROBING -> BUS_OFF transition path).
     * Reason = PSR.BO bit observed set. Discriminates candidate causes
     * 1 + 3 from candidate 2 (probe-side). */
    s_last_bo_trigger_reason = BO_REASON_PSR_BO_BIT;
    bus_state_transition(BUS_OFF);
    return;
  }

  switch (s_bus_state) {
    case BUS_RESETTING:  bus_state_tick_in_RESETTING(); break;
    case BUS_PROBING:    bus_state_tick_in_PROBING();   break;
    case BUS_HEALTHY:    /* no work — patch 0042 runs in smart_md_lockstep_step */ break;
    case BUS_OFF:        bus_state_tick_in_OFF();       break;
    case BUS_LINK_DOWN:  /* terminal — awaits operator reset */ break;
    default: break;
  }
}

/* Unified lockstep entry: called once per handle_data() iteration in main.c.
 * NEVER called from IRQ context. Replaces smart_md_cache_refresh_tick (TIM7),
 * smart_md_handle_rx_frame (HAL_FDCAN_RxFifo0Callback intercept), and
 * smart_md_dispatch_pending_writes (deferred-TX-ring free-drain).
 * Phase 07.4.8 patch 0044: BUS_HEALTHY-gated. Returns immediately when
 * s_bus_state != BUS_HEALTHY (the state machine in smart_md_bus_state_tick
 * owns BO recovery + probe phases). */
__attribute__((noinline))
void smart_md_lockstep_step(void)
{
  /* Boot guard — patch 0026's State guard is superseded by this Instance check
   * (Phase 7.4.5 patch 0040). */
  if (fdcan_2.Instance == (void *)0) {
    return;
  }

  /* Phase 07.4.8 patch 0044 — BUS_HEALTHY-only guard (RESEARCH Pattern 3).
   * The new Layer 2 bus state machine owns BO recovery + bus health
   * preconditions; smart_md_lockstep_step() ONLY runs the patch 0042
   * response-gated lockstep advance when s_bus_state == BUS_HEALTHY.
   * The previous in-lockstep BO preempt block (Bosch AN004 §2 violation —
   * spun on CCCR.INIT, not PSR.ACT) is DELETED by this patch; the state
   * machine's bus_enter_RESETTING + bus_state_tick_in_RESETTING() pair
   * implement the AN004-conformant recovery. */
  if (s_bus_state != BUS_HEALTHY) {
    return;
  }

  /* Idle: no active motors AND no pending writes. */
  bool ring_empty = (s_pending_tx_tail == s_pending_tx_head);
  if (s_poll_active_mask == 0u && ring_empty) {
    return;
  }

  uint32_t t_start = DWT->CYCCNT;

  /* Interleave: every other cycle, drain a write before a poll. */
  if (!ring_empty && (s_lockstep_cycle_counter & 1u)) {
    smart_md_do_write_lockstep();
  } else {
    smart_md_do_poll_lockstep();
  }
  s_lockstep_cycle_counter++;

  uint32_t elapsed_us = (DWT->CYCCNT - t_start) / DWT_CYCLES_PER_US;
  if (elapsed_us > s_handle_data_max_us) {
    s_handle_data_max_us = elapsed_us;
  }
  (void)s_motor_recheck_counter;  /* reserved for periodic offline recheck (Wave 2) */
}


/*===========================================================================
 * HAL_TIM_PeriodElapsedCallback — IRQ context entry point
 *
 * Multiple peripherals may register callbacks for HAL_TIM_PeriodElapsedCallback.
 * We dispatch by handle pointer and return without touching anything else
 * if the call is for a different timer. The HAL design supports this — see
 * stm32h7xx_hal_tim.h for the weak-symbol model.
 *===========================================================================*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim7_smart_md) {
    /* Phase 07.4.5 (patch 0040): TIM7 demoted to 1 Hz liveness watchdog ONLY.
     * No FDCAN access from IRQ context anymore — lockstep state machine in
     * smart_md_lockstep_step() owns the entire FDCAN2 RX FIFO0 from main loop. */
    s_liveness_tick++;

    /* Phase 07.4.5 (patch 0041): per-motor offline-recheck (Wave 2 follow-up).
     * Clear the offline mask + per-motor timeout counters every 1 s so motors
     * that became unresponsive during a transient (boot, 24V power-up race,
     * brief bus stress) get re-probed on the next lockstep cycle. Without
     * this clear, motors that time out 3x during activate stay offline forever.
     * BSS-only writes from IRQ context -- no FDCAN, no HAL calls. Safe. */
    s_motor_offline_mask = 0u;
    for (uint8_t i = 0; i < SMART_MD_MAX_MOTORS; ++i) {
      s_motor_timeout_count[i] = 0u;
    }
    s_motor_recheck_counter++;

    /* Phase 7.4.24 patch 0079 candidate C (H6c fix): NIRQ liveness re-arm.
     * Force NIRQ HIGH if it has been asserted (held LOW) for > 100 ms with a
     * tx_packet still pending. Recovers from stuck-low states caused by H6
     * sub-mechanism (neither NIRQ-precondition nor nCS-precondition
     * dominant — chain 3c routing). T-7.4.24-01 (IWDG livelock) mitigation:
     * 1 Hz TIM7 cadence (NOT super-loop poll). T-7.4.24-03 (masking-without-
     * cure) mitigation via 3 structural differences from reverted patch
     * 0051 D-05: (1) 1 Hz cadence, (2) get_tx_packet_size() > 0 precondition
     * (Pitfall 4), (3) low_duration > 100 ms precondition. Criterion-7 sub-
     * check (a) post/pre ratio-drop falsifies masking-without-cure: if
     * s_nirq_already_low_at_handshake delta < 10 over the 30-s G12, the
     * patch cured; otherwise PARTIAL_FORENSIC routes to Phase 7.4.25
     * Architectural alternative B (CONTEXT D-08).
     * `s_nirq_high_at_cycles` read race-safe per Open Q4: volatile uint32_t,
     * Cortex-M7 word-aligned reads are single-cycle atomic; PRIMASK=1 in
     * writer (system.c:497 dma_handle_data) masks TIM7 interrupt by
     * construction. PHASE_0079C_TIM7_NIRQ_REARM */
    if (is_nirq_low() && (get_tx_packet_size() > 0)) {
      uint32_t now = DWT->CYCCNT;
      uint32_t low_duration_cycles = now - s_nirq_high_at_cycles;
      if (low_duration_cycles > 40000000u) {  /* 100 ms @ 400 MHz */
        set_nirq_high();
        s_nirq_high_at_cycles = now;
        s_nirq_liveness_rearm_count++;
      }
    }
  }
}

/*===========================================================================
 * SPI dispatch hook — peripheral_register_callback(PERIPH_SMART_MD, ...)
 *
 * Mirrors h7_handler at PERIPH_H7=0x09 (the empirically-proven 1500 Hz path).
 * NEVER calls into can_handle_data, fdcan1_handler, or fdcan2_handler. Reads
 * exclusively from s_state_cache, writes response directly via enqueue_packet
 * (PRIMASK-safe).
 *===========================================================================*/

int smart_md_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  /* Phase 07.4.3 Path A (patch 0033) — full-handler DWT timing. */
  uint32_t t_handler_start = DWT->CYCCNT;

  /* Validate frame size first — wire format is fixed by the shared header
   * (Phase 07.4.2 D1: smart_md_command_t == 148 B; was 144 B at Phase 07.4). */
  if (size != sizeof(smart_md_command_t)) {
    s_drops_bad_size++;
    return -1;
  }

  smart_md_command_t cmd;
  memcpy(&cmd, data, sizeof(cmd));

  /* Validate CRC over the leading bytes (everything except the trailing crc32
   * field). Phase 07.4.2 D1: 144 B for the 148 B command; was 140 B at Phase 07.4. */
  uint32_t expected_crc = smart_md_crc32((uint8_t const *)&cmd,
                                         sizeof(smart_md_command_t) - sizeof(uint32_t));
  if (expected_crc != cmd.crc32) {
    s_drops_bad_crc++;
    return -1;
  }

  /* Build response frame. Status default OK; we may downgrade below. */
  smart_md_response_t resp;
  memset(&resp, 0, sizeof(resp));
  resp.opcode          = (uint8_t)(opcode | SMART_MD_OP_RESPONSE);
  resp.status          = SMART_MD_STATUS_OK;
  resp.motor_count     = 0;
  resp._pad0           = 0;
  resp.request_seq     = cmd.request_seq;
  resp.h7_timestamp_us = HAL_GetTick() * 1000u;

  uint32_t now_ms = HAL_GetTick();

  switch (opcode) {
    case SMART_MD_OP_BATCH_READ_STATE: {
      if (cmd.motor_count == 0 || cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      uint8_t found = 0;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        int idx = smart_md_find_motor_index(cmd.md_can_ids[i]);
        if (idx < 0) {
          s_drops_motor_not_found++;
          continue;
        }
        /* Snapshot the cache entry into the response slot. Update cache_age_ms
         * lazily here based on the cached timestamp. */
        motor_state_t snap = s_state_cache[idx];
        uint32_t age_ms = now_ms - (snap.timestamp_us / 1000u);
        if (age_ms > 65535u) age_ms = 65535u;
        snap.cache_age_ms = (uint16_t)age_ms;
        resp.motor_states[found] = snap;
        ++found;
      }
      resp.motor_count = found;
      if (found == 0) {
        resp.status = SMART_MD_STATUS_MOTOR_NOT_FOUND;
      }
    } break;

    case SMART_MD_OP_NOOP: {
      /* Diagnostic round-trip: echo back the cache snapshot for the first
       * known motor so the bench can validate end-to-end with no FDCAN
       * traffic required. */
      if (SMART_MD_KNOWN_MOTOR_COUNT > 0) {
        resp.motor_states[0] = s_state_cache[0];
        resp.motor_count = 1;
      }
    } break;

    case SMART_MD_OP_GET_DIAG: {
      /* Phase 07.4.3 Plan 03 Path A — counter snapshot. Pack the 8 u32
       * diagnostic counters into motor_states[0] (32-byte slot) via a
       * smart_md_diag_payload_t-compatible layout. Userspace casts the
       * received motor_states[0] to smart_md_diag_payload_t to decode.
       *
       * Phase 07.4.5 (patch 0035) — REPURPOSED FIELDS for FDCAN2 visibility:
       *   s_md_writes_acked  (dead since patch 0032 response-wait removal)
       *     → live snapshot of FDCAN2->PSR (Protocol Status Register)
       *       LEC[2:0], ACT[4:3], EP[5], EW[6], BO[7], DLEC[10:8] etc.
       *   s_md_writes_no_ack (dead since patch 0032 response-wait removal)
       *     → live snapshot of FDCAN2->ECR (Error Counter Register)
       *       TEC[7:0] (TX errors), REC[14:8] (RX errors), RP[15],
       *       CEL[23:16] (CAN Error Logging).
       *
       * Purpose: localise the SET_TARGETS 11 Hz bottleneck. If TEC is
       * saturated near 255 or BO is set, the FDCAN bus is in error-passive/
       * bus-off and the rate cap is bus-level (no motor ACK on the wire).
       * Both registers are direct reads; safe whether fdcan_2 is initialised
       * or not (guarded).
       *
       * Phase 07.4.6 (patch 0038) — REPURPOSED FIELDS for encoder feedback:
       *   s_md_writes_total_us → s_poll_emitted_total
       *     (active READ_REGISTER polls TX'd by cache_refresh_tick).
       *   s_md_writes_max_us   → s_poll_responses_drained
       *     (matching RESPONSE_LEGACY frames parsed into s_state_cache).
       *
       * Handler-timing counters (s_md_writes_total_us/max_us) still
       * accumulate in the SET_TARGETS path but are no longer surfaced via
       * GET_DIAG — encoder-feedback observability supersedes them while
       * Plan 08 rate-sweep is gated on confirming active poll is healthy. */
      smart_md_diag_payload_t * diag = (smart_md_diag_payload_t *)&resp.motor_states[0];
      diag->s_md_writes_emitted         = s_md_writes_emitted;
      if (fdcan_2.Instance != (void *)0) {
        diag->s_md_writes_acked         = fdcan_2.Instance->PSR;
        diag->s_md_writes_no_ack        = fdcan_2.Instance->ECR;
      } else {
        diag->s_md_writes_acked         = 0xDEADBEEFu;  /* Instance==NULL sentinel */
        diag->s_md_writes_no_ack        = 0xDEADBEEFu;
      }
      diag->s_md_writes_fdcan_not_ready = s_md_writes_fdcan_not_ready;
      diag->s_md_writes_total_us        = s_poll_emitted_total;     /* repurposed (0038) */
      diag->s_md_writes_max_us          = s_poll_responses_drained; /* repurposed (0038) */
      /* Phase 07.4.8 patch 0044 — REPURPOSED FIELDS for bus state machine
       * observability (RESEARCH Pattern 5 'simplest' packed encoding). The
       * wire-protocol field names stay unchanged (preserves the locked
       * sizeof(smart_md_diag_payload_t) == 32 invariant at
       * cs8h7_smart_md_frame.h:385); the SEMANTICS of the two existing dead
       * slots are repurposed for the Layer 2 state machine.
       *
       *   s_refresh_ticks_total (offset 24..27): 8-bit-per-state state-entry
       *                                          counters (BUS_RESETTING /
       *                                          BUS_PROBING / BUS_HEALTHY /
       *                                          BUS_OFF) — 4 × 8 bits LE.
       *     bits  0..7  = s_bus_state_transitions[BUS_RESETTING]
       *     bits  8..15 = s_bus_state_transitions[BUS_PROBING]
       *     bits 16..23 = s_bus_state_transitions[BUS_HEALTHY]
       *     bits 24..31 = s_bus_state_transitions[BUS_OFF]
       *
       *   s_refresh_frames_drained (offset 28..31): BUS_LINK_DOWN counter +
       *                                             current state + BO backoff_ms.
       *     bits  0..7  = s_bus_state_transitions[BUS_LINK_DOWN]
       *     bits  8..15 = (uint8_t)s_bus_state (current state, 0..4)
       *     bits 16..31 = s_bo_backoff_ms (LE, capped at 0xFFFF — 1000 ms safely fits)
       *
       * Analyzer side: scripts/phase07.4.7-rate-analyze.py
       * decode_bus_state_transitions() unpacks these two 32-bit slots into
       * the G9 + G10 inputs. The repurpose mirrors patch 0035's PSR/ECR
       * pattern (dead s_md_writes_acked/no_ack → live FDCAN2->PSR/ECR) and
       * patch 0038's s_poll_emitted_total / s_poll_responses_drained pattern. */
      diag->s_refresh_ticks_total =
            ((uint32_t)s_bus_state_transitions[BUS_RESETTING] <<  0)
          | ((uint32_t)s_bus_state_transitions[BUS_PROBING]   <<  8)
          | ((uint32_t)s_bus_state_transitions[BUS_HEALTHY]   << 16)
          | ((uint32_t)s_bus_state_transitions[BUS_OFF]       << 24);
      /* Phase 07.4.9 patch 0047 — REPACKED to surface s_motor_absent_mask
       * (G11 evidence) in bits 2..7. Replaces patch 0046's layout.
       *   bits  0.. 1  BUS_LINK_DOWN counter saturated at 3 (G9 only checks ==0)
       *   bits  2.. 7  s_motor_absent_mask                  (6 bits NEW — G11)
       *   bits  8..11  current_state                        (4 bits, UNCHANGED)
       *   bits 12..14  s_last_bo_trigger_reason             (3 bits; was 4)
       *   bits 15..21  s_bus_probe_responses_total          (7 bits; was 8 — saturates at 127)
       *   bits 22..23  reserved                             (2 bits)
       *   bits 24..31  s_bo_backoff_ms / 4                  (8 bits, UNCHANGED)
       * Analyzer side: scripts/phase07.4.7-rate-analyze.py
       * decode_bus_state_transitions() updated to match (D-04 / D-04d).
       * sizeof(smart_md_diag_payload_t) == 32 invariant preserved. */
      {
        uint8_t link_down_pack = (uint8_t)((s_bus_state_transitions[BUS_LINK_DOWN] > 3u)
                                          ? 3u
                                          : s_bus_state_transitions[BUS_LINK_DOWN]);
        uint8_t probe_total_pack = (uint8_t)((s_bus_probe_responses_total > 127u)
                                            ? 127u
                                            : s_bus_probe_responses_total);
        uint8_t backoff_pack = (uint8_t)((s_bo_backoff_ms / 4u) > 255u
                                        ? 255u
                                        : (s_bo_backoff_ms / 4u));
        diag->s_refresh_frames_drained =
              ((uint32_t)(link_down_pack & 0x03u)              <<  0)
            | ((uint32_t)(s_motor_absent_mask & 0x3Fu)         <<  2)
            | ((uint32_t)((uint8_t)s_bus_state & 0x0Fu)        <<  8)
            | ((uint32_t)(s_last_bo_trigger_reason & 0x07u)    << 12)
            | ((uint32_t)(probe_total_pack & 0x7Fu)            << 15)
            | ((uint32_t)backoff_pack                          << 24);
      }
      resp.motor_count = 1;
    } break;

    case SMART_MD_OP_SET_MODE: {
      /* Phase 07.4.3 D2: payload[] is smart_md_set_mode_payload_t.
       * One MD80 WRITE_REGISTER frame per motor — reg 0x140 motionModeCommand u8.
       * Wire payload: [0x40, 0x00, 0x40, 0x01, mode_u8] (5 B; RESEARCH §3.1). */
      if (cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      smart_md_set_mode_payload_t const * p =
          (smart_md_set_mode_payload_t const *)cmd.payload;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x140,
                                     &p->modes[i], sizeof(uint8_t));
      }
      resp.status = SMART_MD_STATUS_OK;
      /* Phase 07.4.17 patch 0067 (D-01): RT-2 NIRQ-handshake-stall drain-on-exit.
       * Heavy IOCTL handlers may leave NIRQ asserted from a partial transaction
       * (set_nirq_low inside the smart_md_emit_md_write -> can_write -> FDCAN2
       * ACK chain). Without this drain, next handshake at main.c:164
       * (!is_nirq_low() && ... > 0) skips TX dispatch and kernel sees
       * smart_md_status=0x0B BUS_NOT_READY. Drain runs inside caller's PRIMASK=1
       * (set at system.c:442); no additional PRIMASK entry needed (RESEARCH Pitfall 1). */
      if (is_nirq_low()) {
        set_nirq_high();
        s_nirq_stuck_at_set_mode++;
      }
      s_last_completed_handler = SMART_MD_LAST_HANDLER_SET_MODE;  /* PHASE_0073_H4_DISAMBIG */
    } break;

    case SMART_MD_OP_SET_IMPEDANCE_GAINS: {
      /* Phase 07.4.12 patch 0050 (D-04): RT-2 plausibility latency probe.
       * Capture DWT cycle count at case-entry; subtract at case-exit and
       * convert to microseconds via existing firmware /DWT_CYCLES_PER_US
       * convention (SYSCLK=400 MHz; see smart_md_handler.c:1129 + 1741 for prior art).
       * Last assignment wins across multiple SET_IMPEDANCE_GAINS calls;
       * activate sequence calls it once per session so the value reflects
       * activate-time measurement. */
      uint32_t const _phase_7412_start_cycles = DWT->CYCCNT;
      /* Phase 07.4.6 — full "prepare motor for IMPEDANCE motion" opcode.
       * Per-motor write sequence:
       *   reg 0x089 runClearWarnings    (u8=1)  — clear latched warnings
       *                                          (incl. CAN watchdog bit 30
       *                                          which blocks ENABLE)
       *   reg 0x08A runClearErrors      (u8=1)  — clear latched errors
       *   reg 0x050 motorImpPidKp       (f32)   — impedance proportional gain
       *   reg 0x051 motorImpPidKd       (f32)   — impedance derivative gain
       *   reg 0x112 maxTorque           (f32)   — motor torque saturation
       * Source-of-truth: md_types.hpp:62-63 (gains), :83-84 (clear), :98 (mtq).
       *
       * Called from OphanimSystem::activate_smart_md_ between SET_MODE and
       * SET_TARGETS hold.  WITHOUT the clear-warnings step, motor stays
       * stuck at state=0x40 (DISABLED) even after subsequent ENABLE writes
       * because the WarningCANWatchdog bit (communicationStatus bit 30)
       * latches whenever the CAN bus is silent for > 250 ms (motor's
       * canWatchdog timeout, reg 0x003).  Empirically observed 2026-05-12
       * on motor 311 with 24V restored: ENABLE writes had no effect on
       * state register until runClearWarnings was issued first. */
      if (cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      smart_md_impedance_gains_payload_t const * gp =
          (smart_md_impedance_gains_payload_t const *)cmd.payload;
      uint8_t const clear_one = 1u;
      uint16_t const watchdog_off = 0u;  /* disable CAN watchdog */
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        /* Preamble: watchdog disable + clearWarnings + clearErrors (PRESERVED
         * verbatim from patch 0038; these are single-register writes that don't
         * need atomic packing with the gains). */
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x003,
                                     &watchdog_off, sizeof(uint16_t));
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x089,
                                     &clear_one, sizeof(uint8_t));
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x08A,
                                     &clear_one, sizeof(uint8_t));

        /* Phase 07.4.5 F1 root-cause fix (BLOCKER-C): emit Kp+Kd as a SINGLE
         * multi-register WRITE_REGISTER_LEGACY frame matching CANdle-SDK
         * MD.cpp:252-264 setImpedanceParams byte layout. The motor's IMPEDANCE
         * control loop applies BOTH gains atomically at a single state-machine
         * boundary. Three-separate-frames pattern (patch 0038) lost atomicity →
         * motor reverted to FLASH gains.
         *
         * Wire payload: [0x40, 0x00, 0x50, 0x00, kp_LE_4B, 0x51, 0x00, kd_LE_4B]
         *   = 14 bytes. DLC=16 promoted by the SocketCAN driver.
         * Source-of-truth: CANdle-SDK MD.hpp:565-567 writeRegisters template
         *   byte emitter (NOT MD.cpp:252-264 which is the setImpedanceParams CALLER —
         *   the Phase 7.4.5 F1 fix referenced the wrong file and missed the padding;
         *   Phase 7.4.10 patch 0048 closes the gap).
         * Slot->value carries the post-opcode-and-padding bytes (12 B): [0x50, 0x00,
         * kp_LE_4B, 0x51, 0x00, kd_LE_4B]. Lockstep TX_WRITE prepends 0x40 + 0x00. */
        uint32_t head = s_pending_tx_head;
        uint32_t next_head = (head + 1u) & (SMART_MD_TX_RING_SIZE - 1u);
        if (next_head == s_pending_tx_tail) {
          s_pending_tx_overflow++;
        } else {
          smart_md_pending_tx_t * slot = &s_pending_tx_ring[head];
          slot->md_can_id  = cmd.md_can_ids[i];
          slot->reg_addr   = 0x050;  /* first reg in the bundle (matcher key for A3 — patch 0058) */
          slot->frame_kind = SMART_MD_FRAME_WRITE_IMPEDANCE_DUAL;
          slot->value_size = 12u;
          uint8_t *p = slot->value;
          p[0] = 0x50; p[1] = 0x00; memcpy(&p[2], &gp->gains[i].kp, 4);
          p[6] = 0x51; p[7] = 0x00; memcpy(&p[8], &gp->gains[i].kd, 4);
          __DMB();
          s_pending_tx_head = next_head;
        }

        /* maxTorque (0x112) is NOT coupled to the IMPEDANCE Kp/Kd atomic
         * boundary — keep as a separate single-register write. */
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x112,
                                     &gp->gains[i].max_torque, sizeof(float));
      }
      resp.status = SMART_MD_STATUS_OK;
      /* Phase 07.4.17 patch 0067 (D-01): RT-2 drain-on-exit (see SET_MODE comment).
       * Inserted BEFORE the s_set_impedance_gains_handler_us latency probe so the
       * probe captures full post-fix case-body cost including the drain (~50 cycles
       * = ~125 ns @ 400 MHz; negligible vs typical handler latency). */
      if (is_nirq_low()) {
        set_nirq_high();
        s_nirq_stuck_at_set_imp_gains++;
      }
      /* Phase 7.4.24 patch 0078: SET_IMPEDANCE_GAINS exit-timestamp anchor
       * AFTER drain, BEFORE latency probe (Assumption A3). PHASE_0078_H6_SUBEVENT */
      s_set_imp_gains_exit_dwt_cycles = DWT->CYCCNT;
      s_last_completed_handler = SMART_MD_LAST_HANDLER_SET_IMP_GAINS;  /* PHASE_0073_H4_DISAMBIG */

      s_set_impedance_gains_handler_us = (DWT->CYCCNT - _phase_7412_start_cycles) / DWT_CYCLES_PER_US;
    } break;

    case SMART_MD_OP_SET_TARGETS: {
      /* Phase 07.4.3 D2: payload[] is smart_md_target_t[motor_count].
       * Per-flag emission to {0x150 pos, 0x151 vel, 0x152 torque, 0x140 mode}.
       * Wire payloads (RESEARCH §3.2 + §4.3):
       *   TORQUE flag   -> reg 0x152 f32 LE -> 8 B [0x40, 0x00, 0x52, 0x01, f32]
       *   VELOCITY flag -> reg 0x151 f32 LE -> 8 B
       *   POSITION flag -> reg 0x150 f32 LE -> 8 B
       *   MODE flag     -> reg 0x140 u8     -> 5 B
       * T-07.4.3-01 mitigation: clamp |target_torque| <= MD80_TORQUE_MAX_SAFE
       * before emit (clamp + emit-clamped, NOT silent drop — avoids stuck
       * "torque set to 0 but reported success" surprise). */
      if (cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      /* PHASE_0073_H4_DISAMBIG: entry-time falsifier sampling for F-04 (H4)
       * and F-05 (H3). Sample is_nirq_low() and FDCAN2 RX FIFO0 occupancy
       * BEFORE any motor-write emission -- the first smart_md_emit_md_write
       * inside the for-loop below shifts NIRQ state and would obscure the
       * entry condition. fdcan_2.Instance->RXF0S is a single read-only
       * peripheral register read; no HAL function call (R-A5 compliant).
       * Also raises s_in_set_targets_dispatch so the retry-path accumulator
       * inside dwt_rx_match_or_timeout knows the caller context. */
      s_in_set_targets_dispatch = 1u;
      /* PHASE_0076_H5_INTRA_BURST: sample-to-sample timing + dispatch counter.
       * Entry-probe. DWT->CYCCNT read is single-cycle on Cortex-M7 (CoreDebug
       * counter @ SYSCLK=400 MHz). Unsigned subtraction handles wrap (Pitfall
       * 3); the defensive `< 2e9` gate skips the first-call uninitialized read
       * (s_set_targets_prev_exit_dwt_cycles == 0u sentinel) and any pathological
       * cross-wrap deltas. EWMA alpha=1/16 (15/16 retain + 1/16 new). */
      {
        uint32_t now = DWT->CYCCNT;
        s_set_targets_total_dispatches++;
        if (s_set_targets_prev_exit_dwt_cycles != 0u) {
          uint32_t delta = now - s_set_targets_prev_exit_dwt_cycles;  /* wrap-safe */
          if (delta < (uint32_t)2000000000u) {
            if (s_set_targets_intra_burst_delta_dwt_cycles_min == 0u ||
                delta < s_set_targets_intra_burst_delta_dwt_cycles_min) {
              s_set_targets_intra_burst_delta_dwt_cycles_min = delta;
            }
            if (delta > s_set_targets_intra_burst_delta_dwt_cycles_max) {
              s_set_targets_intra_burst_delta_dwt_cycles_max = delta;
            }
            s_set_targets_intra_burst_delta_dwt_cycles_p50_ewma =
              (s_set_targets_intra_burst_delta_dwt_cycles_p50_ewma * 15u + delta) >> 4;
          }
        }
      }
      {
        uint8_t was_nirq_low_on_entry = is_nirq_low() ? 1u : 0u;
        if (was_nirq_low_on_entry) {
          s_set_targets_nirq_already_low_on_entry++;
          s_prior_handler_at_set_targets_stuck = s_last_completed_handler;
          /* PHASE_0076_H5_INTRA_BURST: frame-position histogram capture at the
           * stuck-on-entry detection site (BEFORE any emit overwrites
           * s_set_targets_current_frame_position; see Pitfall 5). Decodes
           * 0=POS, 1=VEL, 2=TQ; 0xFF sentinel = no prior burst in flight
           * (e.g., first SET_TARGETS dispatch after boot) -- not counted. */
          s_set_targets_prev_burst_frame_position = s_set_targets_current_frame_position;
          if (s_set_targets_current_frame_position == 0u) {
            s_set_targets_prev_burst_frame_position_pos++;
          } else if (s_set_targets_current_frame_position == 1u) {
            s_set_targets_prev_burst_frame_position_vel++;
          } else if (s_set_targets_current_frame_position == 2u) {
            s_set_targets_prev_burst_frame_position_tq++;
          }
          uint8_t fifo_occ = (uint8_t)(fdcan_2.Instance->RXF0S & FDCAN_RXF0S_F0FL_Msk);
          if (fifo_occ > 0u) {
            s_fdcan2_rx_fifo_occupancy_at_set_targets = fifo_occ;
          }
        }
      }
      smart_md_target_t const * targets = (smart_md_target_t const *)cmd.payload;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        smart_md_target_t const * t = &targets[i];

        if (t->flags & SMART_MD_TARGET_FLAG_MODE) {
          (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x140,
                                       &t->mode, sizeof(uint8_t));
        }
        if (t->flags & SMART_MD_TARGET_FLAG_TORQUE) {
          s_set_targets_current_frame_position = 2u;  /* PHASE_0076_H5_INTRA_BURST: TQ (reg 0x152) */
          float tq = t->target_torque;
          /* T-07.4.3-01 mitigation — bounds check (no math.h dependency). */
          if (tq >  MD80_TORQUE_MAX_SAFE) { tq =  MD80_TORQUE_MAX_SAFE; s_md_writes_torque_clamped++; }
          if (tq < -MD80_TORQUE_MAX_SAFE) { tq = -MD80_TORQUE_MAX_SAFE; s_md_writes_torque_clamped++; }
          (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x152,
                                       &tq, sizeof(float));
        }
        if (t->flags & SMART_MD_TARGET_FLAG_VELOCITY) {
          s_set_targets_current_frame_position = 1u;  /* PHASE_0076_H5_INTRA_BURST: VEL (reg 0x151) */
          (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x151,
                                       &t->target_velocity, sizeof(float));
        }
        if (t->flags & SMART_MD_TARGET_FLAG_POSITION) {
          s_set_targets_current_frame_position = 0u;  /* PHASE_0076_H5_INTRA_BURST: POS (reg 0x150) */
          (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x150,
                                       &t->target_position, sizeof(float));
        }
      }
      resp.status = SMART_MD_STATUS_OK;
      /* PHASE_0077_H5_ACK_BARRIER: Phase 7.4.23 patch 0077 — bounded <= 10
       * iter spin-wait until NIRQ de-asserts. Mitigates T-7.4.23-01 IWDG
       * livelock via iteration count. Mirrors patch 0067 RT-2 drain-on-
       * exit pattern. Body is empty by design — NIRQ is CPU-driven GPIO
       * (system.c:297-312); no SPI/FDCAN hardware handshake fires from
       * spin. The 10-iter time budget (~50 cyc = 125 ns @ 400 MHz) lets
       * the WRITE-ACK chain advance before the existing patch-0067 RT-2
       * drain epilogue force-raises NIRQ. */
      for (uint8_t ack_spin = 0u; ack_spin < 10u; ++ack_spin) {
        if (!is_nirq_low()) break;
        __NOP(); __NOP(); __NOP(); __NOP();
        s_set_targets_ack_barrier_spins++;
      }
      /* Phase 07.4.17 patch 0067 (D-01): RT-2 drain-on-exit (see SET_MODE comment).
       * SET_TARGETS is the hottest write path under sine motion; drain overhead is
       * ~50 cycles (~125 ns @ 400 MHz) per handler call -- negligible vs 200 Hz
       * outer-loop budget. T6 60-s bench validates no regression (CONTEXT D-04). */
      /* PHASE_0072_NIRQ_FALSIFIERS: 5-counter falsifier instrumentation -- see Plan 02 D-04. */
      s_set_targets_nirq_attempts++;
      if (cmd.motor_count > 0u && (s_lockstep_cycle_counter & 1u)) {
        /* Phase 07.4.21 patch 0072 -- Pitfall 1 mitigation: sample lockstep
         * phase at SET_TARGETS entry (cycle&1==1 means poll-phase imminent).
         * Sharpens F-03 interpretation (structural vs probabilistic). */
        s_set_targets_at_poll_phase++;
      }
      if (is_nirq_low()) {
        set_nirq_high();
        s_nirq_stuck_at_set_targets++;
        if (s_poll_active_mask != 0u) {
          s_poll_active_at_set_targets_failure++;
        }
      } else {
        s_set_targets_nirq_succeeded++;
      }
      /* PHASE_0076_H5_INTRA_BURST: stamp exit DWT for next handler's delta
       * computation. Sentinel reset of s_set_targets_current_frame_position
       * is INTENTIONALLY OMITTED here -- the next burst's first emit overwrites
       * it naturally, and resetting to 0xFF would clobber the frame-position
       * preserved across the inter-handler gap that the next entry's stuck-
       * on-entry capture relies on (Pitfall 5). */
      s_set_targets_prev_exit_dwt_cycles = DWT->CYCCNT;
      s_in_set_targets_dispatch = 0u;                                /* PHASE_0073_H4_DISAMBIG */
      s_last_completed_handler  = SMART_MD_LAST_HANDLER_SET_TARGETS; /* PHASE_0073_H4_DISAMBIG */
    } break;

    case SMART_MD_OP_ENABLE: {
      /* Write reg 0x142 (state, u16) = 39 (MD.cpp:66 — undocumented magic
       * from CANdle-SDK; do not paraphrase). 6 B wire payload:
       *   [0x40, 0x00, 0x42, 0x01, 0x27, 0x00]   (RESEARCH §3.3) */
      if (cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      uint16_t enable_value = 39;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x142,
                                     &enable_value, sizeof(uint16_t));
        /* Phase 07.4.6 (patch 0038): mark this motor poll-active so
         * cache_refresh_tick starts emitting READ_REGISTER bulk-reads
         * for it.  Cleared by DISABLE. */
        int mi = smart_md_find_motor_index(cmd.md_can_ids[i]);
        if (mi >= 0 && mi < 8) {
          s_poll_active_mask |= (uint8_t)(1u << mi);
        }
      }
      /* PHASE_0075_WATCHDOG_REENABLE: restore MD80 canWatchdog after activate
       * completes. Phase 7.4.22 Plan 03 (ALL CHAINS) safety deliverable. The
       * smart_md SET_IMPEDANCE_GAINS handler writes canWatchdog=0 (around
       * line 1780-1786, patch 0038) to absorb activate-sequence quiet
       * windows. Without re-enable, motors run forever without host-loss
       * crash protection. See memory:feedback_md80_can_watchdog_disabled.
       *
       * Value 500 ms chosen per Phase 7.4.22 RESEARCH Open Q5 RESOLVED:
       * max intra-activate-burst gap on the wire measured at 50 ms in
       * Phase 7.4.20 teensy candump capture. 500 ms = 10x empirical
       * margin. MAB factory default is 250 ms; we use 500 ms for added
       * safety. Runs AFTER s_poll_active_mask is set above -- activate
       * is fully complete before re-arming the watchdog (RESEARCH
       * Pitfall 3). No PRIMASK/DMB/atomics needed (PRIMASK=1 inherited
       * from dma_handle_data critical section). */
      uint16_t const watchdog_on_ms = 500u;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x003,
                                     &watchdog_on_ms, sizeof(uint16_t));
        s_watchdog_re_enable_emitted++;
      }
      resp.status = SMART_MD_STATUS_OK;
      /* Phase 07.4.17 patch 0067 (D-01): RT-2 drain-on-exit (see SET_MODE comment).
       * ENABLE is the final step in the activate sequence; NIRQ stuck here causes
       * Cycle-3 BUS_NOT_READY per Phase 7.4.16 walking-deeper evidence. Post-fix:
       * s_enable_state_readback_mismatch (patch 0066) should finally fire correctly
       * as activate now completes past ENABLE. */
      if (is_nirq_low()) {
        set_nirq_high();
        s_nirq_stuck_at_enable++;
      }
      s_last_completed_handler = SMART_MD_LAST_HANDLER_ENABLE;  /* PHASE_0073_H4_DISAMBIG */
    } break;

    case SMART_MD_OP_DISABLE: {
      /* Write reg 0x142 (state, u16) = 64 (MD.cpp:86 — undocumented magic).
       * 6 B wire payload:
       *   [0x40, 0x00, 0x42, 0x01, 0x40, 0x00]   (RESEARCH §3.4) */
      if (cmd.motor_count > SMART_MD_MAX_MOTORS) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      uint16_t disable_value = 64;
      for (uint8_t i = 0; i < cmd.motor_count; ++i) {
        (void)smart_md_emit_md_write(cmd.md_can_ids[i], 0x142,
                                     &disable_value, sizeof(uint16_t));
        /* Phase 07.4.6 (patch 0038): clear poll-active bit so
         * cache_refresh_tick stops polling this motor. */
        int mi = smart_md_find_motor_index(cmd.md_can_ids[i]);
        if (mi >= 0 && mi < 8) {
          s_poll_active_mask &= (uint8_t)~(1u << mi);
        }
      }
      /* Phase 07.4.19 patch 0069: DISABLE drain epilogue (Phase 7.4.17 RESEARCH OQ-1).
       * DISABLE is a "light" handler that does not busywait, but NIRQ may remain
       * asserted from a prior heavy IOCTL (SET_IMPEDANCE_GAINS / ENABLE / SET_TARGETS)
       * leaving residual handshake state. This drain mirrors the 4 existing drain
       * epilogues from patch 0067 (RT-2 fix). */
      if (is_nirq_low()) {
        set_nirq_high();
        s_nirq_stuck_at_disable++;
      }
      resp.status = SMART_MD_STATUS_OK;
      s_last_completed_handler = SMART_MD_LAST_HANDLER_DISABLE;  /* PHASE_0073_H4_DISAMBIG */
    } break;

    case SMART_MD_OP_READ_REG_SYNC: {
      /* Phase 07.4.5 (patch 0037) — synchronous MD80 READ_REGISTER from IRQ
       * context.  Sends a [0x41, 0x00, addr_lo, addr_hi, 0..(reg_size-1)] wire
       * frame on FDCAN2 (MdFrameId_E::READ_REGISTER), then busy-waits on the
       * RX FIFO using DWT->CYCCNT (NOT HAL_GetTick — patch 0032 lesson:
       * SysTick is suppressed inside SPI dispatch IRQ context) for up to
       * READ_REG_TIMEOUT_US.  On response, packs the value bytes into
       * resp.motor_states[0]._reserved[0..reg_size-1].
       *
       * NVIC priority guarantee: smart_md_handler runs in SPI dispatch IRQ at
       * higher priority than TIM7 (5) and FDCAN2_IT0 (matched).  TIM7 cannot
       * preempt our busy-wait and steal the response from RX FIFO.
       *
       * Budget: ~5 ms one-shot.  At 5 Mbps data rate one FDCAN-FD response
       * frame (8 B payload) takes <100 µs on the wire, plus motor MCU
       * processing.  5 ms is generous; longer would risk IWDG (16 ms @
       * IWDG_PRESCALER_16 + LSI 32 kHz). */
      if (cmd.motor_count != 1) {
        s_drops_bad_motor_count++;
        resp.status = SMART_MD_STATUS_INVALID_MOTOR_COUNT;
        break;
      }
      smart_md_read_reg_payload_t const * rp =
          (smart_md_read_reg_payload_t const *)cmd.payload;
      uint16_t md_id    = cmd.md_can_ids[0];
      uint16_t reg_addr = rp->reg_addr;
      uint8_t  reg_size = rp->reg_size;
      if (reg_size == 0 || reg_size > 8) {
        resp.status = SMART_MD_STATUS_BAD_PAYLOAD;
        break;
      }
      if (fdcan_2.Instance == (void *)0) {
        resp.status = SMART_MD_STATUS_FDCAN_NOT_READY;
        break;
      }
      if (can_tx_fifo_available(&fdcan_2) == 0) {
        resp.status = SMART_MD_STATUS_TX_FAILED;
        break;
      }

      /* Build READ_REGISTER request frame.
       *   [0]    = 0x41  (MdFrameId_E::READ_REGISTER)
       *   [1]    = 0x00  (pad)
       *   [2..3] = reg_addr little-endian
       *   [4..]  = zero-fill placeholder for value bytes
       * Total wire length = 4 + reg_size (motor overwrites placeholder in
       * its response). */
      uint8_t tx_buf[16] = {0};
      tx_buf[0] = 0x41;
      tx_buf[1] = 0x00;
      tx_buf[2] = (uint8_t)(reg_addr & 0xFFu);
      tx_buf[3] = (uint8_t)((reg_addr >> 8) & 0xFFu);
      uint8_t tx_len = (uint8_t)(4u + reg_size);

      int tx_rc = can_write(&fdcan_2, (uint32_t)md_id, tx_len, tx_buf,
                            X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS);
      if (tx_rc != 0) {
        resp.status = SMART_MD_STATUS_TX_FAILED;
        break;
      }

      /* Busy-wait on FDCAN2 RX FIFO0 for the matching response.  Cycle
       * counter @ SYSCLK=400 MHz: 5000 µs * DWT_CYCLES_PER_US (=400) = 2,000,000 cycles. */
      uint32_t const READ_REG_TIMEOUT_US = 5000u;
      uint32_t const timeout_cycles     = READ_REG_TIMEOUT_US * DWT_CYCLES_PER_US;
      uint32_t const t_start            = DWT->CYCCNT;
      uint8_t  rx_id_low = 0;
      uint8_t  rx_data[64] = {0};
      uint8_t  rx_len = 0;
      uint8_t  rx_flags = 0;
      uint32_t rx_can_id = 0;
      bool     got_match = false;
      while ((DWT->CYCCNT - t_start) < timeout_cycles) {
        if (can_rx_fifo_available(&fdcan_2, FDCAN_RX_FIFO0) == 0) {
          continue;
        }
        if (can_read(&fdcan_2, &rx_can_id, &rx_len, rx_data, &rx_flags) == 0) {
          /* can_read returned 0 — FIFO empty after fill check (race).  Retry. */
          continue;
        }
        /* Must match: source motor CAN ID AND opcode==RESPONSE_LEGACY/READ
         * AND reg addr.  MD80 emits the response with the same CAN ID as
         * the request target (see MD.hpp:440 — drops 2-byte response header,
         * then deserializes addr+value).  Accept either 0xA0 (RESPONSE_LEGACY)
         * or 0x41 (echoed READ_REGISTER) as the response header (md_types.hpp
         * + MD.hpp:428 TODO note re both forms). */
        rx_id_low = (uint8_t)(rx_can_id & 0xFFu);
        (void)rx_id_low;
        if (rx_can_id != md_id) {
          /* Frame from a different motor — drop and keep waiting. */
          continue;
        }
        if (rx_len < (4u + reg_size)) {
          continue;
        }
        if (rx_data[0] != 0xA0 && rx_data[0] != 0x41) {
          continue;
        }
        uint16_t resp_addr = (uint16_t)rx_data[2] | ((uint16_t)rx_data[3] << 8);
        if (resp_addr != reg_addr) {
          continue;
        }
        got_match = true;
        break;
      }

      if (!got_match) {
        resp.status = SMART_MD_STATUS_READ_TIMEOUT;
        break;
      }

      /* Pack the value bytes into motor_states[0]._reserved[0..reg_size-1].
       * Fields position/velocity/torque are not used here (the read may be
       * of any register, not just position). */
      motor_state_t * out = &resp.motor_states[0];
      memset(out, 0, sizeof(*out));
      out->md_can_id    = md_id;
      out->cache_age_ms = 0;  /* fresh read */
      uint8_t copy_n = reg_size;
      if (copy_n > sizeof(out->_reserved)) copy_n = sizeof(out->_reserved);
      memcpy(out->_reserved, &rx_data[4], copy_n);
      resp.motor_count = 1;
      resp.status      = SMART_MD_STATUS_OK;
    } break;

    default: {
      s_drops_bad_opcode++;
      resp.status = SMART_MD_STATUS_INVALID_OPCODE;
    } break;
  }

  /* Phase 07.4.8 patch 0045 — SOFT-LAUNCH BUS_NOT_READY status code (D-04b).
   *
   * When the Layer 2 bus state machine is not in BUS_HEALTHY, surface a
   * BUS_NOT_READY status byte on motor-write opcodes (SET_MODE, SET_TARGETS,
   * SET_IMPEDANCE_GAINS, ENABLE, DISABLE). The H7 still accepts the writes
   * silently — s_md_writes_emitted continues to increment via the underlying
   * smart_md_emit_md_write helper (backward-compatible). The Linux opt-in
   * consumer (deferred per D-04b — added in a later phase via smart_md_node
   * diag log) surfaces this status code. Hard cutover (reject writes outright)
   * is deferred per D-04e to a follow-up phase after soft-launch telemetry
   * indicates which Linux consumers are ready.
   *
   * Status value: 0x0B (next free uint8_t after SMART_MD_STATUS_BAD_RESPONSE = 0x0A
   * in cs8h7_smart_md_frame.h:158; grep at patch-author time confirms 0x08
   * is taken by SMART_MD_STATUS_TX_FAILED, 0x09 by SMART_MD_STATUS_READ_TIMEOUT,
   * 0x0A by SMART_MD_STATUS_BAD_RESPONSE, so 0x0B is the next free value).
   *
   * Defined here (not in the shared header) because the H7 stm32h7-fw submodule
   * patch-series scope cannot modify src/portenta_x8/include/cs8h7_smart_md_frame.h.
   * The Linux-side opt-in consumer must agree on the 0x0B value (deferred —
   * coordinated update to the shared header lands in a follow-up phase per D-04b).
   * The local #define makes the wire value explicit in the firmware build and
   * grep-able by the build-gate symbol assertion (tests/phase07.4-smart-md-build.sh).
   *
   * Override only applies to opcodes that emit motor writes (status reads,
   * GET_DIAG, NOOP continue to report OK so the diag dump remains observable
   * during bus recovery). */
#ifndef SMART_MD_STATUS_BUS_NOT_READY
#define SMART_MD_STATUS_BUS_NOT_READY 0x0Bu
#endif
  if (s_bus_state != BUS_HEALTHY) {
    switch (opcode) {
      case SMART_MD_OP_SET_MODE:
      case SMART_MD_OP_SET_TARGETS:
      case SMART_MD_OP_SET_IMPEDANCE_GAINS:
      case SMART_MD_OP_ENABLE:
      case SMART_MD_OP_DISABLE:
        resp.status = SMART_MD_STATUS_BUS_NOT_READY;
        break;
      default:
        /* Read-side opcodes (BATCH_READ_STATE, GET_DIAG, NOOP, READ_REG_SYNC)
         * keep their original status — the diag dump must remain observable
         * during bus recovery. */
        break;
    }
  }

  /* Compute response CRC over the leading bytes (everything except the trailing
   * crc32 field). Phase 07.4.2 D1: 204 B for the 208 B response; was 140 B at
   * Phase 07.4 (144 B response). */
  resp.crc32 = smart_md_crc32((uint8_t const *)&resp,
                              sizeof(smart_md_response_t) - sizeof(uint32_t));

  /* Publish via enqueue_packet (PRIMASK-safe). The opcode field of the SPI
   * subpacket is the wire-protocol response opcode; the data payload is the
   * full smart_md_response_t. */
  int enqueue_rc = enqueue_packet(PERIPH_SMART_MD, resp.opcode, sizeof(resp), &resp);

  /* Phase 07.4.3 Path A (patch 0033) — capture handler runtime. Reuse the
   * formerly can_write timing counters as handler timing (can_write was
   * confirmed <1µs; investigation moved to full-handler timing). */
  uint32_t t_handler_us = (DWT->CYCCNT - t_handler_start) / DWT_CYCLES_PER_US;
  s_md_writes_total_us += t_handler_us;
  if (t_handler_us > s_md_writes_max_us) {
    s_md_writes_max_us = t_handler_us;
  }
  return enqueue_rc;
}

/*===========================================================================
 * Phase 07.4.5 (patch 0040): smart_md_dispatch_pending_writes RETIRED.
 *
 * The ring-drain function is replaced by smart_md_do_write_lockstep() called
 * once per cycle from smart_md_lockstep_step() (above). The ring data
 * structure is preserved (32 slots × 24 B) for SPI-IRQ producer enqueue;
 * lockstep consumer drains ONE slot per main-loop iteration interleaved with
 * polls (one-frame-in-flight invariant).
 *===========================================================================*/

/*===========================================================================
 * smart_md_handler_init — TIM7 1 Hz liveness config (Phase 07.4.5)
 *
 * TIM7 is a basic timer (16-bit counter, no output channels). Clocked from
 * APB1 timer clock = 240 MHz on the STM32H747 M7 core (default RCC config
 * in this firmware).
 *
 * Phase 07.4.5 (patch 0040): demoted from 1 kHz to 1 Hz liveness watchdog.
 *   Prescaler 24000 -> 10 kHz tick (both factors fit in 16-bit registers);
 *   Period 10000 -> 1 Hz IRQ. TIM7 IRQ does NOT touch FDCAN anymore —
 *   smart_md_lockstep_step() owns the entire FDCAN2 RX FIFO0 from main loop.
 *   IRQ body just increments s_liveness_tick.
 *===========================================================================*/

void smart_md_handler_init(void)
{
  /* Phase 07.4.3 Path A (patch 0033) — enable DWT cycle counter for can_write
   * timing instrumentation. Must enable TRCENA + DWT->CYCCNTENA before reading
   * DWT->CYCCNT. Cortex-M7 has DWT always present; ARM debug spec mandates
   * TRCENA gate. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* Clear the state cache. */
  memset(s_state_cache, 0, sizeof(s_state_cache));
  for (int i = 0; i < SMART_MD_KNOWN_MOTOR_COUNT; ++i) {
    s_state_cache[i].md_can_id    = s_known_md_can_ids[i];
    s_state_cache[i].cache_age_ms = 65535u; /* signals "no fresh data yet" (u16 max) */
  }

  /* Enable TIM7 clock. */
  __HAL_RCC_TIM7_CLK_ENABLE();

  htim7_smart_md.Instance               = TIM7;
  /* Phase 07.4.5 (patch 0040): 1 Hz liveness watchdog only.
   *   240 MHz / 24000 = 10 kHz tick; 10 kHz / 10000 = 1 Hz IRQ.
   * (Both Prescaler and Period must fit in TIM7's 16-bit registers; 24000 and
   * 10000 fit; the direct 1 Hz formulation 240/1000000 does NOT fit.) */
  htim7_smart_md.Init.Prescaler         = 24000 - 1;   /* 240 MHz / 24000 = 10 kHz */
  htim7_smart_md.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim7_smart_md.Init.Period            = 10000 - 1;   /* 10 kHz / 10000 = 1 Hz */
  htim7_smart_md.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim7_smart_md) != HAL_OK) {
    Error_Handler("smart_md_handler_init: HAL_TIM_Base_Init(TIM7) failed.");
    return;
  }

  /* TIM7 IRQ priority — pick a value lower (numerically higher) than FDCAN2
   * (priority 2) so the cache poll cannot starve FDCAN RX. */
  HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim7_smart_md) != HAL_OK) {
    Error_Handler("smart_md_handler_init: HAL_TIM_Base_Start_IT(TIM7) failed.");
    return;
  }

  /* Phase 07.4.8 patch 0043 — Layer 1 exclusive FDCAN2 RX FIFO0 ownership
   * (D-01b DISABLE branch per Plan 01 AUDIT `no_non_md_consumers_found`).
   *
   * Per RESEARCH OQ-1 option (a): flip the ownership flag AFTER smart_md_handler_init
   * has succeeded (TIM7 + DWT + cache zeroed) but BEFORE FDCAN2 is initialised
   * (CAN_INIT opcode arrives at runtime via the SPI bridge). When CAN_INIT later
   * fires for FDCAN2, on_CAN_INIT_Request() observes smart_md_is_active() == true
   * and calls HAL_FDCAN_DeactivateNotification on the RX FIFO0 IT mask (the IRQ
   * prong — Pitfall 3). The main-loop drainer in can_handle_data()::is_can2_init
   * branch also queries smart_md_is_active() and skips the polling drain (the
   * polling prong — Pitfall 3). Both prongs are mandatory; landing only one
   * reproduces the Phase 7.4.6 stolen-response failure shape. */
  s_smart_md_active = true;
}
