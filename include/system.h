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

#ifndef SYSTEM_H
#define SYSTEM_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <inttypes.h>
#include <stdbool.h>

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define SPI_DMA_BUFFER_SIZE   256

__attribute__((packed, aligned(4))) struct subpacket {
  __attribute__((packed, aligned(4))) struct {
    uint8_t peripheral;
    uint8_t opcode;
    uint16_t size;
  } header;
  uint8_t raw_data;
};

__attribute__((packed, aligned(4))) struct complete_packet {
  __attribute__((packed, aligned(4))) struct {
    uint16_t size;
    uint16_t checksum;
  } header;
  struct subpacket data;
  // ... other subpackets will follow
};

#define max(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void system_init();

void dma_init();
void dma_load(bool const swap_tx_buf);

int get_available_enqueue();
int enqueue_packet(uint8_t const peripheral, uint8_t const opcode, uint16_t const size, void * data);
void set_nirq_low();
void set_nirq_high();  /* Phase 07.4.13 patch 0051 (D-03): NIRQ watchdog force-clear — symmetric inverse of set_nirq_low() at system.c:297; declared here so main.c watchdog block can call it without implicit declaration warning */
/* Phase 07.4.14 patch 0054 (A4): unified completion-publish helper.
 * Called by HAL_SPI_TxRxCpltCallback + HAL_SPI_ErrorCallback. RESEARCH §2. */
void smart_md_publish_response_done(void);
bool is_nirq_low();
bool is_ncs_low();
uint16_t get_tx_packet_size();

void dma_handle_data();

/* Phase 07.4.15 patch 0059 (A5 + B2 ROLLED FORWARD from Phase 7.4.14 Stage 4):
 * Clock constants for DWT-based timeout math.
 * DWT->CYCCNT runs at CPU clock = SYSCLK (Cortex-M7 TRM section 11.7).
 * SystemClock_Config (system.c:73-92): HSI=64 -> PLLM=32, PLLN=400, PLLP=2
 *   -> SYSCLK = 64/32 * 400 / 2 = 400 MHz; AHBCLKDivider=DIV2 -> HCLK = 200 MHz.
 * Prior `/480u` divisor was wrong by 20% (treated CPU as 480 MHz). Telemetry
 * calibration only -- no behavioral impact on control logic. B2 wrap-safety:
 * 32-bit DWT->CYCCNT wraps every 4_294_967_296 / 400_000_000 ~= 10.7 s; all
 * timeouts in smart_md_handler.c are <= 5 ms so wrap-safe by construction. */
#define SYSCLK_HZ          400000000u
#define HCLK_HZ            200000000u
#define DWT_CYCLES_PER_US  (SYSCLK_HZ / 1000000u)   /* = 400 */

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })


#endif //SYSTEM_H
