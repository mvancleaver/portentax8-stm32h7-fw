/*
 * Phase 7.4 Plan 02 — smart_md_handler.h
 *
 * H7-resident smart-MD x8h7 channel handler. See src/smart_md_handler.c
 * for the implementation contract. This header is included by main.c
 * (peripheral_register_callback site) and by stm32h7xx_it.c
 * (TIM7_IRQHandler dispatcher).
 *
 * Authoritative references:
 *   .planning/phases/07.4-smart-md-x8h7-channel/07.4-CONTEXT.md (D-A1..D-A8)
 *   .planning/phases/07.4-smart-md-x8h7-channel/07.4-RESEARCH.md (R2 IRQ-context lock)
 *   src/portenta_x8/include/cs8h7_smart_md_frame.h (shared wire-protocol header)
 */

#ifndef PORTENTAX8_STM32H7_FW_SMART_MD_HANDLER_H
#define PORTENTAX8_STM32H7_FW_SMART_MD_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* SPI dispatch hook — registered via peripheral_register_callback(PERIPH_SMART_MD,
 * &smart_md_handler) parallel to peripheral_register_callback(PERIPH_H7, &h7_handler).
 * Returns response via enqueue_packet() (PRIMASK-safe, IRQ-context callable). */
int smart_md_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size);

/* Phase 07.4.8 D-01b/D-01c — exclusive FDCAN2 RX FIFO0 ownership accessor.
 * Returns true once smart_md owns FDCAN2 (post smart_md_handler_init()).
 * Consumed by can_handler.c::can_handle_data() to skip the main-loop fdcan_2
 * polling drain (Pitfall 3 — second prong) and by can_handler.c::on_CAN_INIT_Request
 * to disarm the FDCAN2 RX FIFO0 IRQ notification (Pitfall 3 — first prong). */
bool smart_md_is_active(void);

/* HAL_TIM-based 1 kHz cache-poll initializer. Called from peripheral_init().
 * Configures TIM7 to fire HAL_TIM_PeriodElapsedCallback at 1000 Hz; that
 * callback in turn invokes smart_md_cache_refresh_tick() from IRQ context. */
void smart_md_handler_init(void);

/* Phase 07.4.5 lockstep substrate (patch 0040): unified poll + write dispatch.
 * Replaces smart_md_cache_refresh_tick (TIM7 async path), smart_md_handle_rx_frame
 * (HAL_FDCAN_RxFifo0Callback intercept), and smart_md_dispatch_pending_writes
 * (deferred-TX-ring free-drain). Single frame in flight at all times; DWT-spin
 * with bench-driven timeout SMART_MD_LOCKSTEP_RESPONSE_TIMEOUT_US.
 * Called once per handle_data() iteration from main.c.
 * Phase 07.4.8 patch 0044: now BUS_HEALTHY-gated — returns immediately when
 * the bus state machine is in BUS_RESETTING / BUS_PROBING / BUS_OFF / BUS_LINK_DOWN. */
void smart_md_lockstep_step(void);

/* Phase 07.4.8 patch 0044 — top-level H7 bus state machine tick.
 * Called once per handle_data() iteration from main.c, ORDERED FIRST (before
 * can_handle_data() so the Layer 1 polling gate sees fresh state; before
 * smart_md_lockstep_step() so the BUS_HEALTHY guard reads fresh state).
 * Implements the 5-state machine BUS_RESETTING → BUS_PROBING → BUS_HEALTHY
 * → BUS_OFF → (BUS_RESETTING|BUS_LINK_DOWN) per CONTEXT D-02/D-03.
 * BUS_RESETTING entry is Bosch M_CAN AN004 §2 conformant — polls PSR.ACT==1
 * (NOT spinning on CCCR.INIT, RESEARCH §"Pitfall 1"; replaces the broken
 * in-lockstep BO preempt that 0042 carried forward). */
void smart_md_bus_state_tick(void);

/* Exposed handle for stm32h7xx_it.c TIM7_IRQHandler. */
extern TIM_HandleTypeDef htim7_smart_md;

#endif /* PORTENTAX8_STM32H7_FW_SMART_MD_HANDLER_H */
