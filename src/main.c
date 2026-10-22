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

#include <stdio.h>

#include "peripherals.h"
#include "can.h"
#include "can_handler.h"
#include "rpc.h"
#include "adc.h"
#include "adc_handler.h"
#include "uart.h"
#include "uart_handler.h"
#include "virtual_uart.h"
#include "virtual_uart_handler.h"
#include "pwm.h"
#include "pwm_handler.h"
#include "gpio.h"
#include "gpio_handler.h"
#include "timer.h"
#include "rtc.h"
#include "rtc_handler.h"
#include "spi.h"
#include "system.h"
#include "h7_handler.h"
#include "smart_md_handler.h"
#include "watchdog.h"
#include "m4_util.h"

/**************************************************************************************
 * DIAGNOSTICS
 **************************************************************************************/

/* Phase 07.4.12 patch 0050 (D-02): RT-2 NIRQ-handshake-stall counter.
 * Increments when the SPI bridge handshake check would want to assert NIRQ
 * but finds NIRQ already low from a prior response not yet drained by the
 * Linux side. Per rtos-engineer REVIEW RT-2: if > 0 after the activate
 * sequence, the NIRQ-stall hypothesis is VERIFIED. Declared in main.c per
 * D-02 (file-scope BSS in src/main.c); no extern needed — build gate reads
 * value via openocd at the BSS address resolved from post-patch-syms.txt. */
static volatile uint32_t s_nirq_already_low_at_handshake = 0;

/* Phase 07.4.14 patch 0056 (A6): NIRQ rapid-pulse race instrumentation +
 * minimum-NIRQ-high enforcement. RESEARCH §1 — at estimated 200-1500 ns
 * natural pulse width, i.MX8M Mini synchronizer (66 MHz IPG, ~30 ns
 * threshold) latches reliably most of the time, but residual short-pulse
 * conditions (post-A4 cleanup of stranded TX) may still occur. The 5 µs
 * threshold (= 2000 DWT cycles @ 400 MHz CPU clock — NOT 480 MHz; RESEARCH
 * §6 correction) is well above the upper-bound natural pulse; counter
 * increments only when a pulse is force-extended.
 * Verdict per RESEARCH §1.4: INCONCLUSIVE-bench-required, RECOMMEND LAND.
 * Counter is the falsifier: zero post-bench means A6 was unnecessary-
 * but-harmless; >0 means the race was real.
 * `s_nirq_high_at_cycles` is set in dma_handle_data (system.c:468 after
 * set_nirq_high()) via an extern declaration; cleared here when we pull low.
 * 2000u literal raw NOT DWT_CYCLES_PER_US macro because Stage 4 (patch 0059)
 * has not landed yet. */
static volatile uint32_t s_nirq_high_pulse_short = 0;
volatile uint32_t s_nirq_high_at_cycles = 0;

/* Phase 7.4.24 patch 0078 (H6 sub-event disambiguation): 6 sub-event counters
 * at main.c:164-193 to discriminate H6a (NIRQ-precondition) vs H6b
 * (nCS-precondition) vs H6c (both/race). T-7.4.24-02 mitigation: ~60 ns
 * added inside existing PRIMASK=1 critical section; total ~125 ns vs the
 * existing ~75 ns helper cost; <2% of NIRQ HIGH pulse width. Read-once
 * GPIO snapshots (Pitfall 5).
 * PHASE_0078_H6_SUBEVENT */
static volatile uint32_t s_nirq_gate_total_calls            = 0;
static volatile uint32_t s_nirq_gate_nirq_already_low_only  = 0;
static volatile uint32_t s_nirq_gate_ncs_low_only           = 0;
static volatile uint32_t s_nirq_gate_both_low               = 0;
static volatile uint32_t s_nirq_gate_unexpected             = 0;
static volatile uint32_t s_nirq_gate_no_tx_packet           = 0;

/* 2026-10-22 (07.4.42.2 bench, 600 s prodmix FAIL at +108 s): NIRQ handshake self-heal.
 * The H7 asserts NIRQ LOW once (below) and only releases it after the X8's exchange has been
 * processed (dma_handle_data). If the i.MX8 misses that falling edge (sub-us re-pulse race, A6)
 * the H7 waits forever with a swapped TX buffer armed; the X8 only rescues it with an exchange of
 * its own, and when its TX ring is full waiting for the TX_COMPLETE stuck in that very buffer
 * nothing moves until the 50 ms netdev watchdog ("TX stalled (lost H7 status?)"). Here: if NIRQ
 * has been LOW for X8H7_NIRQ_REPULSE_US with TX data pending and nCS never went LOW (no exchange
 * started), re-pulse NIRQ (HIGH >= 5 us, then LOW) to regenerate the edge. Harmless if the X8 was
 * merely late (IRQF_ONESHOT: a second edge while its thread runs is ignored). Counted. */
#ifndef X8H7_NIRQ_REPULSE_US
#define X8H7_NIRQ_REPULSE_US 500u
#endif
static volatile uint32_t s_nirq_low_at_cycles   = 0;   /* DWT->CYCCNT when we last pulled NIRQ low */
static volatile uint32_t s_nirq_repulse_total   = 0;   /* re-pulses issued */

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void peripheral_init()
{
  peripheral_register_callback(PERIPH_H7, &h7_handler);

  /* Phase 7.4.25 Arch-B (patch 0080): smart_md_handler_init() and PERIPH_SMART_MD
   * registration REMOVED. H7 is now a thin SPI ↔ FDCAN2 raw-frame relay; all MD80
   * protocol lives in CANdle-SDK on the X8 host. smart_md_is_active() still
   * resolves (returns false because s_smart_md_active static is never set true),
   * so can_handler.c:219 `!smart_md_is_active()` gate uniformly succeeds and
   * FDCAN2 RX FIFO0 drain takes over without code change. See RESEARCH §Architecture
   * Patterns Pattern 1. */

  uart_init();
  peripheral_register_callback(PERIPH_UART, &uart_handler);

  virtual_uart_init();
  peripheral_register_callback(PERIPH_VIRTUAL_UART, &virtual_uart_handler);

  gpio_init();
  peripheral_register_callback(PERIPH_GPIO, &gpio_handler);

  pwm_init();
  peripheral_register_callback(PERIPH_PWM, &pwm_handler);

  dma_init();

  timer_init();

  rtc_init();
  peripheral_register_callback(PERIPH_RTC, &rtc_handler);

  spi_init();

  dma_load(false);

  adc_init();
  peripheral_register_callback(PERIPH_ADC, &adc_handler);

  peripheral_register_callback(PERIPH_FDCAN1, &fdcan1_handler);
  peripheral_register_callback(PERIPH_FDCAN2, &fdcan2_handler);
}

void handle_data()
{
  __WFI();

  watchdog_refresh();

  /* Phase 7.4.25 Arch-B (patch 0080): smart_md_bus_state_tick() and
   * smart_md_lockstep_step() REMOVED — H7 no longer runs the MD80
   * protocol state machine; FDCAN2 RX FIFO0 drain is owned by the
   * existing can_handle_data() path (gated `!smart_md_is_active()`). */

  if (uart_data_available())
    uart_handle_data();

  if (virtual_uart_data_available())
    virtual_uart_handle_data();

  can_handle_data();
  gpio_handle_data();
  dma_handle_data();

  {
    /* Enter critical section. */
    uint32_t primask_bit = __get_PRIMASK();
    __set_PRIMASK(1) ;

    /* Phase 7.4.25 Arch-B (patch 0080): minimal NIRQ-HIGH→LOW transition
     * path preserved (Pitfall 5 mitigation). Patch-0056 5 µs hold guard
     * kept verbatim. The 6-way H6 sub-event classifier (patch 0078) and
     * s_nirq_already_low_at_handshake increment (patch 0050) DELETED with
     * the smart-md ENABLE-pathway; their BSS counters remain declared
     * above (D-A3 carry-forward) but read 0 at runtime now. */
    if (!is_nirq_low() && !is_ncs_low() && (get_tx_packet_size() > 0))
    {
      /* Phase 07.4.14 patch 0056 (A6): enforce minimum 5 µs NIRQ HIGH
       * duration before pulling LOW. 2000u = 5 µs * 400 MHz CPU clock. */
      if (s_nirq_high_at_cycles != 0) {
        if ((DWT->CYCCNT - s_nirq_high_at_cycles) < 2000u) {
          while ((DWT->CYCCNT - s_nirq_high_at_cycles) < 2000u) { /* busy-wait */ }
          s_nirq_high_pulse_short++;
        }
      }
      dma_load(true);
      set_nirq_low();
      s_nirq_high_at_cycles = 0;
      s_nirq_low_at_cycles  = DWT->CYCCNT;
    }
    else if (is_nirq_low() && !is_ncs_low() && (get_tx_packet_size() > 0 || s_nirq_low_at_cycles != 0))
    {
      /* NIRQ asserted, X8 has not started an exchange: after X8H7_NIRQ_REPULSE_US regenerate the edge. */
      uint32_t const cycles_per_us = SystemCoreClock / 1000000u;
      if (s_nirq_low_at_cycles != 0 &&
          (DWT->CYCCNT - s_nirq_low_at_cycles) > (X8H7_NIRQ_REPULSE_US * cycles_per_us))
      {
        set_nirq_high();
        uint32_t const t0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - t0) < (5u * cycles_per_us)) { /* >= 5 us HIGH (patch 0056 minimum) */ }
        set_nirq_low();
        s_nirq_low_at_cycles = DWT->CYCCNT;
        s_nirq_repulse_total++;
      }
    }

    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
  }
}

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(void)
{
  gpio_init_nirq();

  system_init();

  peripheral_init();

  disableCM4Autoboot();

  extern char const REAL_VERSION_FLASH[];
  printf("Portenta X8 - STM32H7 companion fw - %s\n", REAL_VERSION_FLASH);

  try_execute_m4_app();

  watchdog_init(IWDG_PRESCALER_16);

  gpio_init_ncs();

  for(;;) {
    handle_data();
  }

  /* Never return. */
}
