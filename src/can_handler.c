/*
 * Firmware for the Portenta X8 STM32H747AIIX/Cortex-M7 core.
 * Copyright (C) 2022 Arduino (http://www.arduino.cc/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "can_handler.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#include "can.h"
#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "peripherals.h"
/* Phase 07.3 Plan 03 patch 0024 — fdcan_rx_to_rpmsg() hook for the
 * parallel cs8h7-can OpenAMP endpoint. Implementation lives in src/rpc.c
 * next to the existing endpoint registry. */
#include "can_rpmsg_bridge.h"
/* Phase 07.4.6 (patch 0038) — smart_md_handle_rx_frame() intercept hook
 * for FDCAN2 RX frames.  See header for design rationale. */
#include "smart_md_handler.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning
#define X8H7_CAN_STS_FLG_TX_OVR  0x80  // Transmit Buffer Overflow

#define X8H7_CAN_STS_INT_TX_COMPLETE       0x01
#define X8H7_CAN_STS_INT_RX                0x02
#define X8H7_CAN_STS_INT_ERR               0x04
#define X8H7_CAN_STS_INT_TX_ABORT_COMPLETE 0x08
#define X8H7_CAN_STS_INT_TX_FIFO_EMPTY     0x10

/* Phase 07.4.5 (patch 0036) — compile-gate the Phase 06.2 Plan 03c H7-T5/T6
 * OPCODE_TRACE emits.  These debug traces fire on every FDCAN TX completion
 * (incl. every smart-MD write), wasting ~30 B of SPI bandwidth per can_write
 * AND triggering the matching A53-side pr_info (now gated by x8h7/0015 in
 * the kernel module, but the wasted SPI traffic and per-packet dispatch
 * cost persist on the wire).
 *
 * 07.4.42.2 P1-2: also gates H7-T2/T2.5/T3/T4/T12/T13 — 27+ bytes per TX
 * frame in the 252-byte H7→X8 payload that must carry the motor ACKs.
 * Defined here (before the first user) so every per-frame trace sees it.
 *
 * Default DEBUG_H7_TX_TRACE = 0.  Set to 1 (-DDEBUG_H7_TX_TRACE=1) to restore
 * Phase 06.2 trace instrumentation for future FDCAN TX-path debugging.
 * Companion to kernel patch x8h7/0015 (gates the matching pr_info in pkt_parse). */
#ifndef DEBUG_H7_TX_TRACE
#define DEBUG_H7_TX_TRACE 0
#endif

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

union x8h7_can_init_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
    uint32_t data_baud_rate_prescaler;
    uint32_t data_time_segment_1;
    uint32_t data_time_segment_2;
    uint32_t data_sync_jump_width;
  } field;
  uint8_t buf[8 * sizeof(uint32_t)];
};
_Static_assert(sizeof(union x8h7_can_init_message) == 32,
               "x8h7 CAN init wire layout drift");

union x8h7_can_bittiming_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_filter_message
{
  struct __attribute__((packed))
  {
    uint32_t idx;
    uint32_t id;
    uint32_t mask;
  } field;
  uint8_t buf[sizeof(uint32_t) /* idx */ + sizeof(uint32_t) /* id */ + sizeof(uint32_t) /* mask */];
};

union x8h7_can_frame_message
{
  struct __attribute__((packed))
  {
    uint32_t id;                           // 29 bit identifier
    /* WIRE FORMAT INVARIANT (Phase 06.5 — paired with x8h7_can.h):
     * `len` is the BYTE COUNT (0..64). NOT an FDCAN DLC code.
     * Kernel side: copies cfd->len into field.len after WARN_ON +
     *   clamp to X8H7_CAN_FRAME_MAX_DATA_LEN (64). See x8h7_can.c:91-93.
     * H7 side: can_write() switches on len at can.c:316-335. Lengths
     *   not in {0..8, 12, 16, 20, 24, 32, 48, 64} fall through `default:`
     *   to FDCAN_DLC_BYTES_0 (frame transmits with DLC=0). Per ISO 11898-1
     *   §10.4.1 these are the only valid CAN-FD byte counts; non-standard
     *   lengths SHOULD never appear post-kernel-clamp.
     * Wire format = byte count throughout. DLC-code mapping lives ONLY
     *   inside can_write()'s switch. */
    uint8_t  len;                          // Length of data field in bytes
    uint8_t  flags;                        // CAN-FD flags (X8H7_CAN_FLAG_FD|BRS|ESI)
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN]; // Data field
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};
_Static_assert(sizeof(union x8h7_can_frame_message) == 70,
               "x8h7 CAN FD frame wire layout drift");
_Static_assert(X8H7_CAN_FRAME_MAX_DATA_LEN == 64,
               "Phase 06.5 wire-format invariant: ISO 11898-1 max FD payload is 64 bytes; "
               "change requires bilateral wire-format audit (kernel x8h7_can.h + H7 can_handler.c)");

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

extern FDCAN_HandleTypeDef fdcan_1;
extern FDCAN_HandleTypeDef fdcan_2;

static bool is_can1_init = false;
static bool is_can2_init = false;
static uint32_t can_1_last_enqueue = 0;
static uint32_t can_2_last_enqueue = 0;

/* Phase 07.4.42.2: 1 = an error-state report could not be enqueued (SPI payload full); retried from can_handle_data(). */
static volatile uint8_t s_err_state_pending[2] = {0, 0};

/* Phase 07.4.42.2 (RESEARCH P0-4/P1-3): exact per-peripheral TX accounting for the kernel ring.
 * finished = submitted - popcount(TXBRP) (completed + aborted + rejected); reported via
 * {INT_TX_COMPLETE, n_finished, n_aborted}. Updated with PRIMASK set; retried from can_handle_data().
 * Index k = 0 for fdcan_1, 1 for fdcan_2. The sticky TXBTO mask handed to the TX complete callback is NOT
 * used: its bits stay set until the buffer is reused, so a popcount of it over-counts. */
static volatile uint32_t s_tx_submitted[2], s_tx_reported[2], s_tx_aborted[2], s_tx_aborted_reported[2];
#define X8H7_TX_ABORT_SPIN_LIMIT 4000u   /* bounded wait for an abort to free a FIFO slot (no cycle counter under Arch-B) */
void fdcan_tx_accounting_reset(FDCAN_HandleTypeDef * handle);

/* Phase 07.4.42.2 (RESEARCH P1-1): RX frames dropped on the H7 (FIFO0 message lost, or popped from the FIFO but the
 * SPI payload was full). Incremented by the RX FIFO0 ISR and (PRIMASK-guarded) by the can_handle_data() drain;
 * reported as {INT_ERR, RX_OVR, n_lost} once per main-loop pass. Zeroed by fdcan_tx_accounting_reset(). */
static volatile uint32_t s_rx_lost[2], s_rx_lost_reported[2];

/* Report the CURRENT FDCAN error state as {INT_ERR, flags, TEC, REC} (wire contract row "error state").
 * flags come from PSR, not from the interrupt bits, because EP/EW/BO interrupts fire on both entry and exit.
 * Called from HAL_FDCAN_ErrorStatusCallback (FDCANx_IT0 ISR) and, for a pending retry, from can_handle_data()
 * inside a PRIMASK critical section. Reads PSR/ECR only; never restarts the controller. */
static void fdcan_report_error_state(FDCAN_HandleTypeDef * handle)
{
  int const k = (handle == &fdcan_1) ? 0 : 1;
  FDCAN_ProtocolStatusTypeDef ps;
  FDCAN_ErrorCountersTypeDef ec;
  if (HAL_FDCAN_GetProtocolStatus(handle, &ps) != HAL_OK || HAL_FDCAN_GetErrorCounters(handle, &ec) != HAL_OK) {
    s_err_state_pending[k] = 1;
    return;
  }
  uint8_t flags = 0;
  if (ps.BusOff)       flags |= X8H7_CAN_STS_FLG_TX_BO;
  if (ps.ErrorPassive) flags |= (X8H7_CAN_STS_FLG_TX_EP | X8H7_CAN_STS_FLG_RX_EP);
  if (ps.Warning)      flags |= X8H7_CAN_STS_FLG_EWARN;
  uint8_t const tec = (ec.TxErrorCnt > 255u) ? 255u : (uint8_t)ec.TxErrorCnt;
  uint8_t const rec = (ec.RxErrorCnt > 127u) ? 127u : (uint8_t)ec.RxErrorCnt;
  uint8_t x8_msg[4] = { X8H7_CAN_STS_INT_ERR, flags, tec, rec };
  s_err_state_pending[k] = enqueue_packet(k == 0 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg) ? 0 : 1;
}

/* Phase 07.4.42.2: zero the TX accounting of one peripheral. Called by can_init() (before HAL_FDCAN_Init) and
 * can_deinit() (after HAL_FDCAN_DeInit): both empty the Tx FIFO, so the counters restart from zero together with
 * the kernel ring (which is reset on open / CAN_MODE_START / stop). */
void fdcan_tx_accounting_reset(FDCAN_HandleTypeDef * handle)
{
  int const k = (handle == &fdcan_1) ? 0 : 1;
  uint32_t const primask_bit = __get_PRIMASK();
  __set_PRIMASK(1);
  s_tx_submitted[k] = 0;
  s_tx_reported[k] = 0;
  s_tx_aborted[k] = 0;
  s_tx_aborted_reported[k] = 0;
  s_rx_lost[k] = 0;
  s_rx_lost_reported[k] = 0;
  __set_PRIMASK(primask_bit);
}

/* Phase 07.4.42.2 (wire contract row "TX finished", 3 B): report every frame that finished since the last
 * successful report as {INT_TX_COMPLETE, n_finished, n_aborted}. Count-based and therefore idempotent: if the SPI
 * payload is full nothing is marked reported and the next call (TX ISR or the can_handle_data() retry) sends the
 * accumulated delta. Returns the number of frames reported (0 = nothing new or enqueue failed). */
static uint32_t fdcan_report_tx_finished(FDCAN_HandleTypeDef * handle)
{
  int const k = (handle == &fdcan_1) ? 0 : 1;
  uint32_t const primask_bit = __get_PRIMASK();
  __set_PRIMASK(1);
  uint32_t const pending = (uint32_t)__builtin_popcount(handle->Instance->TXBRP);
  uint32_t delta = (s_tx_submitted[k] - pending) - s_tx_reported[k];
  if ((int32_t)delta <= 0)
  {
    __set_PRIMASK(primask_bit);
    return 0;
  }
  if (delta > 255u) delta = 255u;
  uint32_t ab = s_tx_aborted[k] - s_tx_aborted_reported[k];
  if (ab > delta) ab = delta;
  uint8_t x8_msg[3] = { X8H7_CAN_STS_INT_TX_COMPLETE, (uint8_t)delta, (uint8_t)ab };
  if (enqueue_packet(k == 0 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg) > 0)
  {
    s_tx_reported[k] += delta;
    s_tx_aborted_reported[k] += ab;
  }
  else
  {
    delta = 0;
  }
  __set_PRIMASK(primask_bit);
  return delta;
}

/* Phase 07.4.42.2 (wire contract row "RX lost", 3 B): report RX drops not yet reported as
 * {INT_ERR, RX_OVR, n_lost} (n clamped to 255; the rest goes in the next pass). Called once per main-loop pass
 * from can_handle_data() per initialised handle, so the report is aggregated and rate-limited by construction.
 * A MESSAGE_LOST interrupt counts one loss per episode (the flag does not count frames): n_lost is a lower bound. */
static void fdcan_report_rx_overflow(FDCAN_HandleTypeDef * handle)
{
  int const k = (handle == &fdcan_1) ? 0 : 1;
  uint32_t const unreported = s_rx_lost[k] - s_rx_lost_reported[k];
  if (!unreported) return;
  uint32_t const n = (unreported > 255u) ? 255u : unreported;
  uint8_t x8_msg[3] = { X8H7_CAN_STS_INT_ERR, X8H7_CAN_STS_FLG_RX_OVR, (uint8_t)n };
  if (enqueue_packet(k == 0 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg))
    s_rx_lost_reported[k] += n;
}

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

static int fdcan_handler(FDCAN_HandleTypeDef * handle, uint8_t const opcode, uint8_t const * data, uint16_t const size);
static int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width, uint32_t const data_baud_rate_prescaler, uint32_t const data_time_segment_1, uint32_t const data_time_segment_2, uint32_t const data_sync_jump_width);
static int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle);
static int on_CAN_SET_BITTIMING_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width);
static int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask);
static int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int can_handle_data()
{
  int bytes_enqueued = 0;
  uint32_t can_id = 0;
  uint8_t can_len = 0;
  uint8_t can_flags = 0;
  uint8_t can_data[X8H7_CAN_FRAME_MAX_DATA_LEN] = {0};

  /* Note: the last read package is lost in this implementation. We need to fix this by
   * implementing some peek method or by buffering messages in a ringbuffer.
   */
  int rc_enq = 1;

  uint32_t const now = HAL_GetTick();
  /* Phase 07.1.5 Task 2 (Tier 1A bridge cadence fix):
   * The previous 10 ms throttle was the load-bearing root cause of the
   * 95 ms motor 311 readMainState RTT (per bridge-decompose 117-cycle
   * measurement, 2026-05-05). For single-frame replies, the FDCAN RX
   * FIFO0 ISR (HAL_FDCAN_RxFifo0Callback) is gated on watermark=2 and
   * never fires; this slow polling path is the only delivery channel,
   * and the 10 ms gate distributed across all 4 bridge stages.
   * Setting to 0 means "deliver every frame on every main-loop tick"
   * (sub-millisecond on H7). See Plan 07.1.5-01 INVESTIGATION-NOTES.md. */
  static uint32_t const RX_CAN_FRAME_SYNC_TIMEOUT = 0;

  /* Phase 07.4.42.2 (regenerated 0091): retry an error-state report that found the SPI payload full.
   * PRIMASK (saved/restored — enqueue_packet nests the same way) so the FDCAN ISR cannot report a newer
   * state between this PSR read and this enqueue: the kernel must receive the latest state last. */
  if ((is_can1_init && s_err_state_pending[0]) || (is_can2_init && s_err_state_pending[1]))
  {
    uint32_t const primask_bit = __get_PRIMASK();
    __set_PRIMASK(1);
    if (is_can1_init && s_err_state_pending[0]) fdcan_report_error_state(&fdcan_1);
    if (is_can2_init && s_err_state_pending[1]) fdcan_report_error_state(&fdcan_2);
    __set_PRIMASK(primask_bit);
  }

  /* Phase 07.4.42.2 (P0-4): report any TX finished count a TX ISR could not enqueue (SPI payload full) —
   * a lost TX_COMPLETE status must not wedge the kernel ring. No-op (one TXBRP read) when nothing is new. */
  if (is_can1_init) (void)fdcan_report_tx_finished(&fdcan_1);
  if (is_can2_init) (void)fdcan_report_tx_finished(&fdcan_2);

  if (is_can1_init)
  {
    bool const can_1_rx_fifo_frames_available = can_rx_fifo_available(&fdcan_1, FDCAN_RX_FIFO0) > 0;
    bool const can_1_should_enqueue = (now - can_1_last_enqueue) > RX_CAN_FRAME_SYNC_TIMEOUT; /* At least every 10 ms CAN frames should be synced to the X8. */

    if (can_1_rx_fifo_frames_available && can_1_should_enqueue)
    {
      union x8h7_can_frame_message x8h7_msg;

      for (rc_enq = 0;
           (get_available_enqueue() >= sizeof(x8h7_msg)) && can_read(&fdcan_1, &can_id, &can_len, can_data, &can_flags);
           bytes_enqueued += rc_enq)
      {
        x8h7_msg.field.id = can_id;
        x8h7_msg.field.len = can_len;
        x8h7_msg.field.flags = can_flags;
        memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

        rc_enq = enqueue_packet(PERIPH_FDCAN1, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
        if (!rc_enq)
        {
          /* 07.4.42.2 P1-1: can_read already popped the frame — it is lost. PRIMASK: the RX ISR also increments. */
          int const k = 0;
          uint32_t const pm = __get_PRIMASK(); __set_PRIMASK(1); s_rx_lost[k]++; __set_PRIMASK(pm);
          break;
        }

        /* Phase 07.3 Plan 03 patch 0024 — additionally forward to the
         * cs8h7-can RPMsg endpoint (Option A parallel deployment per D-A2).
         * The SPI bridge enqueue_packet above is unaffected by this call;
         * if the RPMsg endpoint is unbound or send fails, the helper
         * silently drops + increments a counter. R-A5 mitigation. */
        fdcan_rx_to_rpmsg(can_id, can_len, can_flags, can_data);

        can_1_last_enqueue = now;
      }
    }
  }

  if (is_can2_init && !smart_md_is_active())
  {
    /* Phase 07.4.8 patch 0043 — Layer 1 exclusive FDCAN2 RX FIFO0 ownership
     * (D-01b DISABLE branch). Pitfall 3 SECOND prong (the main-loop drain
     * skip — see on_CAN_INIT_Request above for the IRQ-notification prong).
     * When smart_md owns FDCAN2 the lockstep cycle-counter spin matcher in
     * dwt_rx_match_or_timeout() must see its own motor responses; the
     * non-MD-aware drain here would steal them. Both prongs are mandatory
     * per Plan 01 AUDIT §6 + RESEARCH §"Pitfall 3"; the kernel-side
     * PERIPH_FDCAN2 CAN_RX_FRAME sink is dormant in smart-MD operating
     * mode per Plan 01 AUDIT §1 verdict, so disabling this drainer does
     * not regress any non-bench / non-MD code path. */
    bool const can_2_rx_fifo_frames_available = can_rx_fifo_available(&fdcan_2, FDCAN_RX_FIFO0) > 0;
    bool const can_2_should_enqueue = (now - can_2_last_enqueue) > RX_CAN_FRAME_SYNC_TIMEOUT; /* At least every 10 ms CAN frames should be synced to the X8. */

    if (can_2_rx_fifo_frames_available && can_2_should_enqueue)
    {
      union x8h7_can_frame_message x8h7_msg;

      for (rc_enq = 0;
           (get_available_enqueue() >= sizeof(x8h7_msg)) && can_read(&fdcan_2, &can_id, &can_len, can_data, &can_flags);
           bytes_enqueued += rc_enq)
      {
        x8h7_msg.field.id = can_id;
        x8h7_msg.field.len = can_len;
        x8h7_msg.field.flags = can_flags;
        memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

        rc_enq = enqueue_packet(PERIPH_FDCAN2, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
        if (!rc_enq)
        {
          /* 07.4.42.2 P1-1: can_read already popped the frame — it is lost. PRIMASK: the RX ISR also increments. */
          int const k = 1;
          uint32_t const pm = __get_PRIMASK(); __set_PRIMASK(1); s_rx_lost[k]++; __set_PRIMASK(pm);
          break;
        }

        can_2_last_enqueue = now;
      }
    }
  }

  /* Phase 07.4.42.2 (P1-1): aggregated RX drop report (FIFO message lost + SPI-full drops). */
  if (is_can1_init) fdcan_report_rx_overflow(&fdcan_1);
  if (is_can2_init) fdcan_report_rx_overflow(&fdcan_2);

  return bytes_enqueued;
}

int fdcan1_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  dbg_printf("fdcan1_handler\n");
#if DEBUG_H7_TX_TRACE
  /* Phase 06.2 Plan 03c — H7-T2 trace. Captures BEFORE the silent-drop
   * guard so we can distinguish "handler entered but is_can1_init=0"
   * from "handler reached fdcan_handler". */
  {
    uint8_t trace_buf[5] = { 2, opcode, (uint8_t)is_can1_init, 0, 0 };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
  }
#endif
  if (!is_can1_init && opcode != CAN_INIT) return 0;
  else return fdcan_handler(&fdcan_1, opcode, data, size);
}

int fdcan2_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  dbg_printf("fdcan2_handler\n");
  if (!is_can2_init && opcode != CAN_INIT) return 0;
  else return fdcan_handler(&fdcan_2, opcode, data, size);
}

int fdcan_handler(FDCAN_HandleTypeDef * handle, uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CAN_INIT)
  {
    dbg_printf("fdcan_handler: CAN_INIT\n");
    union x8h7_can_init_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    return on_CAN_INIT_Request(handle,
                               x8h7_msg.field.baud_rate_prescaler,
                               x8h7_msg.field.time_segment_1,
                               x8h7_msg.field.time_segment_2,
                               x8h7_msg.field.sync_jump_width,
                               x8h7_msg.field.data_baud_rate_prescaler,
                               x8h7_msg.field.data_time_segment_1,
                               x8h7_msg.field.data_time_segment_2,
                               x8h7_msg.field.data_sync_jump_width);
  }
  else if (opcode == CAN_DEINIT)
  {
    dbg_printf("fdcan_handler: CAN_DEINIT\n");
    return on_CAN_DEINIT_Request(handle);
  }
  else if (opcode == CAN_SET_BITTIMING)
  {
    dbg_printf("fdcan_handler: CAN_SET_BITTIMING\n");
    union x8h7_can_bittiming_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    return on_CAN_SET_BITTIMING_Request(handle,
                                        x8h7_msg.field.baud_rate_prescaler,
                                        x8h7_msg.field.time_segment_1,
                                        x8h7_msg.field.time_segment_2,
                                        x8h7_msg.field.sync_jump_width);
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));
    dbg_printf("fdcan_handler: CAN_FILTER\n");
    return on_CAN_FILTER_Request(handle,
                                 x8h7_msg.field.idx,
                                 x8h7_msg.field.id,
                                 x8h7_msg.field.mask);
  }
  else if (opcode == CAN_TX_FRAME)
  {
    /* Phase 06.2 Plan 03c cycle 2 — zero-init defends against the
     * stack-garbage hypothesis (audit §5 #5). Wire payload is 10 bytes
     * for a 4-byte cansend; rest of the 70-byte msg buffer must be
     * deterministically zero or downstream reads of msg.field.flags
     * etc. are undefined. */
    union x8h7_can_frame_message msg = {0};
    memcpy(&msg, data, size);
#if DEBUG_H7_TX_TRACE
    /* Phase 06.2 Plan 03c cycle 2 — H7-T2.5 raw-byte trace. Dumps the
     * actual wire bytes at offsets 3..6 of msg.buf. Per the wire
     * invariant these should be 00 04 00 DE for cansend 100#DEADBEEF. */
    {
      uint8_t trace_buf[5] = { 7, msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6] };
      enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
    }
#endif
    dbg_printf("fdcan_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return on_CAN_TX_FRAME_Request(handle, &msg);
  }
  else
  {
    dbg_printf("fdcan_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle,
                        uint32_t const baud_rate_prescaler, uint32_t const time_segment_1,
                        uint32_t const time_segment_2, uint32_t const sync_jump_width,
                        uint32_t const data_baud_rate_prescaler, uint32_t const data_time_segment_1,
                        uint32_t const data_time_segment_2, uint32_t const data_sync_jump_width)
{
  /* Phase 07.4.42.2: a fresh init starts error-active — drop any report pending from the previous session.
   * Cleared BEFORE can_init() so a report raised by the new session (IRQ armed in can_internal_init) survives. */
  s_err_state_pending[(handle == &fdcan_1) ? 0 : 1] = 0;

  can_init(handle,
           (handle == &fdcan_1) ? CAN_1 : CAN_2,
           baud_rate_prescaler,
           time_segment_1,
           time_segment_2,
           sync_jump_width,
           data_baud_rate_prescaler,
           data_time_segment_1,
           data_time_segment_2,
           data_sync_jump_width);

  if      (handle == &fdcan_1) is_can1_init = true;
  else if (handle == &fdcan_2) is_can2_init = true;

  /* Phase 07.4.8 patch 0043 — Layer 1 exclusive FDCAN2 RX FIFO0 ownership
   * (D-01b DISABLE branch per Plan 01 AUDIT `no_non_md_consumers_found`).
   *
   * Pitfall 3 first prong: disarm the FDCAN2 RX FIFO0 IRQ notification once
   * smart_md owns the bus. can_init() in can.c re-arms FDCAN_IT_RX_FIFO0_*
   * unconditionally; we deactivate it immediately after a successful CAN_INIT
   * if smart_md is active. The polling-side prong lives in can_handle_data()
   * below. Both prongs are mandatory (Plan 01 AUDIT §6); landing only the IRQ
   * leaves the main-loop drainer at can_handle_data():229 stealing smart-MD
   * responses (Phase 7.4.6 R5: s_poll_responses_drained = 0 while emits >> 0).
   * (See smart_md_handler.c for s_smart_md_active lifecycle — RESEARCH OQ-1
   * option (a) — flag flipped at end of smart_md_handler_init().) */
  if (handle == &fdcan_2 && smart_md_is_active()) {
    (void)HAL_FDCAN_DeactivateNotification(&fdcan_2,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE
      | FDCAN_IT_RX_FIFO0_FULL
      | FDCAN_IT_RX_FIFO0_WATERMARK);
  }

  return 0;
}

int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle)
{
  can_deinit(handle);

  if      (handle == &fdcan_1) is_can1_init = false;
  else if (handle == &fdcan_2) is_can2_init = false;

  /* Phase 07.4.42.2: no error-state retry for a de-initialised controller (IRQ disabled by can_deinit). */
  s_err_state_pending[(handle == &fdcan_1) ? 0 : 1] = 0;

  return 0;
}

int on_CAN_SET_BITTIMING_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width)
{
  return can_set_bittiming(handle,
                           baud_rate_prescaler,
                           time_segment_1,
                           time_segment_2,
                           sync_jump_width);
}

int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask)
{
  if (!can_filter(handle, filter_index, id, mask, id & CAN_EFF_FLAG))
    dbg_printf("fdcan2_handler: can_filter failed for idx: %ld, id: %lX, mask: %lX\n", filter_index, id, mask);
  return 0;
}

int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg)
{
#if DEBUG_H7_TX_TRACE
  /* Phase 06.2 Plan 03c — H7-T3 trace. Captures handler entry + TX FIFO
   * availability BEFORE we attempt the HAL submit. v0=fifo_avail (low byte),
   * v1=msg.len, v2=msg.flags. */
  {
    uint32_t const fifo_avail = can_tx_fifo_available(handle);
    uint8_t trace_buf[5] = { 3, (uint8_t)(fifo_avail & 0xFF), msg->field.len, msg->field.flags, (handle == &fdcan_1) ? 1 : 2 };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
  }
#endif

  int const k = (handle == &fdcan_1) ? 0 : 1;
  uint8_t const periph = (k == 0) ? PERIPH_FDCAN1 : PERIPH_FDCAN2;

  if (!can_tx_fifo_available(handle))
  {
    /* 07.4.42.2 P1-3: abort the OLDEST pending element — a control target older than one cycle is
     * worthless, and with no ACK on the bus AutoRetransmission would hold it forever. TXFQS.TFGI is the
     * Tx FIFO get index (oldest pending element). This runs inside dma_handle_data()'s PRIMASK section, so
     * the wait for the freed slot is iteration-bounded (an element mid-transmission is cancelled at the end
     * of the current attempt) and never blocks. The abort is reported through fdcan_report_tx_finished()
     * (abort callback or main-loop retry) as finished + aborted. */
    uint32_t const gi = (handle->Instance->TXFQS & FDCAN_TXFQS_TFGI_Msk) >> FDCAN_TXFQS_TFGI_Pos;
    if (HAL_FDCAN_AbortTxRequest(handle, 1UL << gi) == HAL_OK)
    {
      uint32_t const pm_abort = __get_PRIMASK(); __set_PRIMASK(1); s_tx_aborted[k]++; __set_PRIMASK(pm_abort);
      for (uint32_t spin = 0; spin < X8H7_TX_ABORT_SPIN_LIMIT && !can_tx_fifo_available(handle); ++spin) { __NOP(); }
    }
  }

  if (!can_tx_fifo_available(handle))
  {
    /* Still full: reject this frame. Rejected = finished + aborted (it never set TXBRP), so the kernel ring
     * retires it through the TX finished count; the 3-byte TX_OVR is informational (kernel must not retire on it). */
    uint32_t const pm_rej = __get_PRIMASK(); __set_PRIMASK(1); s_tx_submitted[k]++; s_tx_aborted[k]++; __set_PRIMASK(pm_rej);
    uint8_t x8_msg[3] = {X8H7_CAN_STS_INT_ERR, X8H7_CAN_STS_FLG_TX_OVR, 0};
    (void)enqueue_packet(periph, CAN_STATUS, sizeof(x8_msg), x8_msg);
    (void)fdcan_report_tx_finished(handle);
    return 0;
  }

  /* Submit and count atomically: no TX ISR may compute submitted - popcount(TXBRP) between the TXBAR write and
   * the increment. A failed can_write never set TXBRP, so counting it as submitted (+ aborted) makes it
   * "finished" immediately. */
  uint32_t const pm = __get_PRIMASK(); __set_PRIMASK(1);
  int const rc = can_write(handle, msg->field.id, msg->field.len, msg->field.data, msg->field.flags);
  s_tx_submitted[k]++;
  if (rc < 0) s_tx_aborted[k]++;   /* submit failed: finished + aborted */
  __set_PRIMASK(pm);

  /* Phase 06.3 cycle 6 — H7-T12 trace. Dumps FDCAN1 PSR/CCCR/TXBRP/TXFQS
   * and the RCC FDCANSEL kernel-clock source field IMMEDIATELY after the
   * HAL_FDCAN_AddMessageToTxFifoQ call returns (wrapped by can_write).
   * Three back-to-back 5-byte records (T12.A=PSR, T12.B=CCCR,
   * T12.C=TXBRP+TXFQS+FDCANSEL). Decision matrix in patches/stm32h7-fw/
   * 0012-canfd-trace-fdcan-registers.patch header. Only fires for fdcan_1
   * (fdcan_2 is structurally identical and currently unused). */
#if DEBUG_H7_TX_TRACE
  if (handle == &fdcan_1)
  {
    uint32_t const psr      = FDCAN1->PSR;
    uint32_t const cccr     = FDCAN1->CCCR;
    uint32_t const txbrp    = FDCAN1->TXBRP;
    uint32_t const txfqs    = FDCAN1->TXFQS;
    uint32_t const fdcansel = (RCC->D2CCIP1R & RCC_D2CCIP1R_FDCANSEL_Msk) >> RCC_D2CCIP1R_FDCANSEL_Pos;
    uint8_t trace_a[5] = { 12, 0xA, (uint8_t)(psr   & 0xFF), (uint8_t)((psr   >> 8) & 0xFF), (uint8_t)((psr   >> 16) & 0xFF) };
    uint8_t trace_b[5] = { 12, 0xB, (uint8_t)(cccr  & 0xFF), (uint8_t)((cccr  >> 8) & 0xFF), (uint8_t)((cccr  >> 16) & 0xFF) };
    uint8_t trace_c[5] = { 12, 0xC, (uint8_t)(txbrp & 0xFF), (uint8_t)(txfqs  & 0xFF),       (uint8_t)(fdcansel & 0x3) };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_a), trace_a);
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_b), trace_b);
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_c), trace_c);
  }
#endif

  /* Phase 06.3 cycle 7 — H7-T13 trace. Dumps GPIOD->{MODER, AFR[0],
   * OTYPER, OSPEEDR} field slices for PD0 (FDCAN1_RX) and PD1
   * (FDCAN1_TX) at runtime. Discriminates "GPIO got reverted to plain
   * IO mode somewhere" (hypothesis #3) from "GPIO config is correct
   * and bug is hardware-side" (hypotheses #1 U1 TX dead / #2 PD1↔U1
   * trace fault). 4 back-to-back 5-byte records. Decision matrix in
   * patches/stm32h7-fw/0013-canfd-trace-gpio-runtime.patch header.
   * Only fires for fdcan_1. */
#if DEBUG_H7_TX_TRACE
  if (handle == &fdcan_1)
  {
    uint32_t const moder   = GPIOD->MODER;
    uint32_t const afrl    = GPIOD->AFR[0];
    uint32_t const otyper  = GPIOD->OTYPER;
    uint32_t const ospeedr = GPIOD->OSPEEDR;
    uint8_t const pd0_mode  = (uint8_t)((moder   >> 0) & 0x3);
    uint8_t const pd1_mode  = (uint8_t)((moder   >> 2) & 0x3);
    uint8_t const pd0_af    = (uint8_t)((afrl    >> 0) & 0xF);
    uint8_t const pd1_af    = (uint8_t)((afrl    >> 4) & 0xF);
    uint8_t const pd0_otype = (uint8_t)((otyper  >> 0) & 0x1);
    uint8_t const pd1_otype = (uint8_t)((otyper  >> 1) & 0x1);
    uint8_t const pd0_speed = (uint8_t)((ospeedr >> 0) & 0x3);
    uint8_t const pd1_speed = (uint8_t)((ospeedr >> 2) & 0x3);
    uint8_t trace_0[5] = { 13, 0, (uint8_t)(pd0_mode | (pd0_af << 2)), 0, 0 };
    uint8_t trace_1[5] = { 13, 1, (uint8_t)(pd1_mode | (pd1_af << 2)), 0, 0 };
    uint8_t trace_2[5] = { 13, 2, pd0_otype, pd0_speed, 0 };
    uint8_t trace_3[5] = { 13, 3, pd1_otype, pd1_speed, 0 };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_0), trace_0);
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_1), trace_1);
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_2), trace_2);
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_3), trace_3);
  }
#endif

#if DEBUG_H7_TX_TRACE
  /* Phase 06.2 Plan 03c — H7-T4 trace. Captures the can_write return code
   * (which is the negated HAL_FDCAN_GetError() value, or 0 on success). */
  {
    int8_t rc8 = (rc < -127) ? -127 : (int8_t)rc;
    uint8_t trace_buf[5] = { 4, (uint8_t)rc8, (uint8_t)((uint32_t)(-rc) & 0xFF), (uint8_t)(((uint32_t)(-rc) >> 8) & 0xFF), (uint8_t)(((uint32_t)(-rc) >> 16) & 0xFF) };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
  }
#endif

  if (rc < 0)
  {
    /* Per-frame submit failure (NOT error-passive, RESEARCH P1-4). Informational 3-byte status: the frame is
     * retired through the TX finished count (counted as submitted + aborted above). */
    uint8_t x8_msg[3] = {X8H7_CAN_STS_INT_ERR, X8H7_CAN_STS_FLG_TX_EP, 0};
    (void)enqueue_packet(periph, CAN_STATUS, sizeof(x8_msg), x8_msg);
    (void)fdcan_report_tx_finished(handle);
  }
  return 0;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef * handle, uint32_t BufferIndexes)
{
#if DEBUG_H7_TX_TRACE
  /* Phase 06.2 Plan 03c — H7-T5 trace. Confirms FDCAN actually transmitted
   * the frame on the wire and the TX-complete IRQ fired. */
  {
    uint8_t trace_buf[5] = { 5, (uint8_t)(BufferIndexes & 0xFF), (uint8_t)((BufferIndexes >> 8) & 0xFF), (handle == &fdcan_1) ? 1 : 2, 0 };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
  }
#endif

  /* Phase 07.4.42.2 (P0-4): count-based report; BufferIndexes (sticky TXBTO & TXBTIE) is not used for accounting. */
  uint32_t const reported = fdcan_report_tx_finished(handle);
  (void)reported;  /* consumed only by H7-T6 trace below when DEBUG_H7_TX_TRACE */

#if DEBUG_H7_TX_TRACE
  /* Phase 06.2 Plan 03c — H7-T6 trace. Confirms the IRQ-back STATUS reply
   * was appended to the SPI TX buffer. v0=frames reported by this call
   * (07.4.42.2: count-based; 0 = nothing new or SPI payload full). */
  {
    uint8_t const rep8 = (reported > 255u) ? 255u : (uint8_t)reported;
    uint8_t trace_buf[5] = { 6, rep8, (uint8_t)(BufferIndexes & 0xFF), 0, 0 };
    enqueue_packet(PERIPH_H7, OPCODE_TRACE, sizeof(trace_buf), trace_buf);
  }
#endif
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef * handle, uint32_t BufferIndexes)
{
  /* Phase 07.4.42.2 (P0-4/P1-3): an aborted element is reported in the TX finished count (as aborted);
   * the new firmware no longer sends the legacy per-mask TX_ABORT_COMPLETE status. */
  (void)BufferIndexes;
  (void)fdcan_report_tx_finished(handle);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef * handle)
{
  uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_TX_FIFO_EMPTY, can_tx_fifo_available(handle)};
  enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint32_t can_id = 0;
  uint8_t can_len = 0;
  uint8_t can_flags = 0;
  uint8_t can_data[X8H7_CAN_FRAME_MAX_DATA_LEN] = {0};
  int const k = (hfdcan == &fdcan_1) ? 0 : 1;

  /* Phase 07.4.42.2 (P1-1): RF0L — the FIFO was full and a frame was discarded (blocking mode). Armed in
   * can_internal_init(); one count per interrupt (episode), reported by can_handle_data(). */
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) s_rx_lost[k]++;

  /* Phase 07.4.6 (patch 0038) — also drain on NEW_MESSAGE so smart-MD
   * responses (typically 1 frame at a time, well under the watermark=2
   * threshold) get parsed promptly into s_state_cache.  Before this fix,
   * single frames sat in FIFO until a second frame arrived and hit
   * watermark.  Plan 07.1.5 already activated NEW_MESSAGE notification
   * (can.c:164) but the callback never matched on it. */
  if(((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) ||
      (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) ||
      (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) != RESET)
  {
    union x8h7_can_frame_message x8h7_msg;

    while ((get_available_enqueue() >= sizeof(x8h7_msg)) && can_read(hfdcan, &can_id, &can_len, can_data, &can_flags))
    {
      x8h7_msg.field.id = can_id;
      x8h7_msg.field.len = can_len;
      x8h7_msg.field.flags = can_flags;
      memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

      /* Phase 07.4.5 (patch 0040) — smart-MD intercept REMOVED.
       * smart_md_lockstep_step() in main.c owns FDCAN2 RX FIFO0 polling for
       * smart-MD known motor IDs from non-IRQ context. Leaving the intercept
       * in place would race the lockstep poll-wait. Forwarding to the kernel
       * x8h7_can queue below is unchanged for non-smart-MD bus traffic. */

      int ret = enqueue_packet(hfdcan == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
      if (ret == 0) { s_rx_lost[k]++; break; }   /* 07.4.42.2 P1-1: popped but not forwarded — lost */

      /* Phase 07.3 Plan 03 patch 0024 — ISR-side hook for the cs8h7-can
       * RPMsg endpoint (FDCAN1 only; FDCAN2 = carrier-hat external bus is
       * out of scope for this endpoint per Plan 7.3-03 wiring). Same
       * Option A parallel-deployment semantics as the polling-drain hook
       * in can_handle_data — silent drop on unbound/send-fail. */
      if (hfdcan == &fdcan_1) {
        fdcan_rx_to_rpmsg(can_id, can_len, can_flags, can_data);
      }

      /* Save the timestamp when we've last enqueued data using the RX FIFO watermark. */
      if (hfdcan == &fdcan_1)
        can_1_last_enqueue = HAL_GetTick();
      else
        can_2_last_enqueue = HAL_GetTick();
    }

    /* Phase 07.1.5 Task 2 (Tier 1A bridge cadence fix):
     * Re-arm with NEW_MESSAGE so subsequent single frames also fire the ISR
     * after we drained FIFO0 here. Mask must match can.c initial activation
     * (07.4.42.2: incl. MESSAGE_LOST). */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK)
    {
      /* Notification Error */
    }
  }
}

/* Phase 07.4.42.2 (regenerated 0091): overrides the weak HAL callback; armed in can_internal_init(). */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef * handle, uint32_t ErrorStatusITs)
{
  if (ErrorStatusITs & (FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING))
    fdcan_report_error_state(handle);
}
