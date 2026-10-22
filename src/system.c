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

#include "system.h"
#include "error_handler.h"
#include "debug.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "rpc.h"
#include "spi.h"
#include "gpio.h"
#include "stm32h7xx_ll_exti.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer_1        [SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer_2        [SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer          [SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer_userspace[SPI_DMA_BUFFER_SIZE];

volatile bool is_rx_buf_userspace_processed = false;

volatile uint8_t * p_tx_buf_active   = TX_Buffer_1;
volatile uint8_t * p_tx_buf_transfer = TX_Buffer_2;
volatile struct subpacket * rx_pkt_userspace = (struct subpacket *)RX_Buffer_userspace;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void SystemClock_Config(void) {

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler("HAL_RCC_OscConfig failed.");
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler("HAL_RCC_ClockConfig failed.");
  }

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

static void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 128;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler("HAL_RCCEx_PeriphCLKConfig failed.");
  }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Do MPU */
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer_1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer_2;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer_userspace;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void clean_dma_buffer()
{
  memset((uint8_t*)TX_Buffer_1, 0, sizeof(TX_Buffer_1));
  memset((uint8_t*)TX_Buffer_2, 0, sizeof(TX_Buffer_2));
  memset((uint8_t*)RX_Buffer, 0, sizeof(RX_Buffer));
  memset((uint8_t*)RX_Buffer_userspace, 0, sizeof(RX_Buffer_userspace));

  struct complete_packet * pkt = (struct complete_packet *)TX_Buffer_1;
  pkt->header.size = 0;
  pkt->header.checksum = pkt->header.size ^ 0x5555;

  pkt = (struct complete_packet *)TX_Buffer_2;
  pkt->header.size = 0;
  pkt->header.checksum = pkt->header.size ^ 0x5555;
}

int get_available_enqueue()
{
  /* Enter critical section. */
  uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1) ;

  struct complete_packet * pkt = (struct complete_packet *)p_tx_buf_active;
  /* 07.4.42.2 REVIEW WR-04: payload bytes the NEXT enqueue_packet() can take,
   * i.e. net of its 4-byte subpacket header (callers compare this against the
   * payload size they are about to enqueue). Clamped at 0. */
  int const num_bytes_available = (SPI_DMA_BUFFER_SIZE - 4) - (int)pkt->header.size
                                  - (int)sizeof(((struct subpacket *)0)->header);

  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);

  return num_bytes_available > 0 ? num_bytes_available : 0;
}

int enqueue_packet(uint8_t const peripheral, uint8_t const opcode, uint16_t const size, void * data)
{
  /* Enter critical section: Since this function is called both from inside
   * interrupt context (gpio_handle_irq/gpio.c) as well as from normal execution
   * context it is necessary not only to blindly re-enable interrupts, but
   * to store the current interrupt situation and restore it at the end of
   * the critical section.
   */
  volatile uint32_t primask_bit = __get_PRIMASK();
  /*
   * PRIMASK - Typically configured in code using the CMSIS __disable_irq() and
   * __enable_irq() routines or the cpsid i and cpsie i assembly instructions
   * directly. Setting the PRIMASK to 1 disables all exceptions of configurable
   * priority. This means, only NMI, Hardfault, & Reset exceptions can still occur.
   *
   * Source: https://interrupt.memfault.com/blog/arm-cortex-m-exceptions-and-nvic
   */
  __set_PRIMASK(1) ;

  int bytes_enqueued = 0;

  /* complete_packet:
   * - uint16_t size;      |
   * - uint16_t checksum;  | sizeof(complete_packet.header) = 4 Bytes
   */
  struct complete_packet * pkt = (struct complete_packet *)p_tx_buf_active;
  struct subpacket subpkt;
  /* 07.4.42.2 REVIEW WR-04: the subpacket written below is its 4-byte header
   * plus `size` payload bytes; the bound used to omit the header, so a packet
   * with header.size + size in 249..252 ran up to 4 bytes past the 252-byte
   * data region (mirror of the kernel x8h7_pkt_enq() fix, x8h7 f1e0ee9). */
  if ((pkt->header.size + sizeof(subpkt.header) + size) > (SPI_DMA_BUFFER_SIZE - 4))
    goto cleanup;

  /* subpacket:
   * - uint8_t peripheral; |
   * - uint8_t opcode;     |
   * - uint16_t size;      | sizeof(subpacket.header) = 4 Bytes
   * - uint8_t raw_data;
   */
  subpkt.header.peripheral = peripheral;
  subpkt.header.opcode = opcode;
  subpkt.header.size = size;
  /* Copy subpacket.header at the end of the current complete_packet superframe. */
  memcpy((uint8_t*)&(pkt->data) + pkt->header.size, &subpkt, sizeof(subpkt.header));
  pkt->header.size += sizeof(subpkt.header);
  /* Copy subpacket.raw_data at after subpacket.header. */
  memcpy((uint8_t*)&(pkt->data) + pkt->header.size, data, size);
  pkt->header.size += size;
  /* Calculate a simple checksum to ensure bit flips in the length field can be recognized. */
  pkt->header.checksum = pkt->header.size ^ 0x5555;
  /* Update internal status variable of how many bytes have been enqueued. */
  bytes_enqueued += sizeof(subpkt.header) + size;

#ifdef DEBUG
  char dbg_msg[64] = {0};
  snprintf(dbg_msg, sizeof(dbg_msg), "%s op: %02X size: %d", peripheral_to_string(peripheral), opcode, size);

  char data_msg[64] = {0};
  uint16_t data_msg_len = 0;

  for (uint16_t i = 0; i < size && i < sizeof(data_msg); i++)
    data_msg_len += snprintf(data_msg + data_msg_len, sizeof(data_msg) - data_msg_len, "%02X ", *(((uint8_t*)data) + i));

  dbg_printf("enqueue_packet: %s data: %s\n", dbg_msg, data_msg);
#endif

cleanup:

  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);

  return bytes_enqueued;
}

/* Trigger transfer. */
void set_nirq_low()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
}

/* Signal completion of transfer. */
void set_nirq_high()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

/* Check if there is an ongoing transfer. */
bool is_nirq_low()
{
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
}

bool is_ncs_low()
{
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET);
}

uint16_t get_tx_packet_size()
{
  struct complete_packet * tx_pkt = (struct complete_packet *)p_tx_buf_active;
  uint16_t const tx_packet_size = tx_pkt->header.size;
  return tx_packet_size;
}

/* 2026-10-22: data waiting in EITHER buffer (armed transfer buffer or the accumulating active one). */
bool tx_data_pending(void)
{
  struct complete_packet * a = (struct complete_packet *)p_tx_buf_active;
  struct complete_packet * x = (struct complete_packet *)p_tx_buf_transfer;
  return (a->header.size > 0) || (x->header.size > 0);
}

void system_init() {

  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  PeriphCommonClock_Config();
}

void dma_init()
{
  MX_DMA_Init();
  clean_dma_buffer();
}

extern SPI_HandleTypeDef hspi3;

volatile uint32_t s_dma_load_failures = 0;
void dma_load(bool const swap_tx_buf)
{
  /* 2026-10-22: abort only if a transfer is actually armed/in flight. From the completion callback the
   * SPI is READY and HAL_SPI_Abort would only burn time (it waits on FIFO flush timeouts). */
  if (hspi3.State != HAL_SPI_STATE_READY)
    HAL_SPI_Abort(&hspi3);

  if (swap_tx_buf)
  {
    p_tx_buf_transfer = p_tx_buf_active;
    p_tx_buf_active = (p_tx_buf_active == TX_Buffer_1) ? TX_Buffer_2 : TX_Buffer_1;
  }

  struct complete_packet *tx_pkt = (struct complete_packet *)p_tx_buf_transfer;
  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;

  uint8_t * tx_buf = (uint8_t*)&(tx_pkt->header);
  uint8_t * rx_buf = (uint8_t*)&(rx_pkt->header);

  HAL_StatusTypeDef const rc = HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf, rx_buf, SPI_DMA_BUFFER_SIZE);
  if (rc != HAL_OK) {
    s_dma_load_failures++;   /* 2026-10-22: counted, not printed (console UART print = tens of ms stall) */
#ifdef DEBUG
    dbg_printf("HAL_SPI_TransmitReceive_DMA failed with %d, spi error code = 0x%lX\n", rc, HAL_SPI_GetError(&hspi3));
#endif
  }
}

void EXTI15_10_IRQHandler(void)
{
  /* PA15 = nCS */
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET && LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_15)) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  } else {
    gpio_handle_irq();
  }
}

/* Phase 07.4.14 patch 0054 (A4): unified completion-publish helper.
 * RESEARCH §2 — three sites converge through this:
 *   1. HAL_SPI_TxRxCpltCallback (normal completion) — refactored below
 *   2. HAL_SPI_ErrorCallback (error completion) — hardened below (latent bug §2.4)
 *   3. Any future force-clear path (Stage 1 D-05 watchdog already reverted)
 * Side-effect: increments s_response_published_total (A4 falsifier; should
 * be 1:1 with s_md_writes_emitted post-fix per RESEARCH §10.2 G16-bridge). */
static volatile uint32_t s_response_published_total = 0;

void smart_md_publish_response_done(void)
{
  struct complete_packet *tx_pkt = (struct complete_packet *)p_tx_buf_transfer;
  tx_pkt->header.size     = 0;
  tx_pkt->header.checksum = tx_pkt->header.size ^ 0x5555;
  s_response_published_total++;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;

  /* Limit the amount of data copied to prevent buffer overflow.
   * Phase 06.2 Plan 03c cycle 2 fix: previously used sizeof(rx_pkt_userspace)
   * which is the POINTER size (4 bytes), not the BUFFER size. */
  if (rx_pkt->header.size > sizeof(RX_Buffer_userspace))
    rx_pkt->header.size = sizeof(RX_Buffer_userspace);

  /* The SPI transfer is now complete, copy to userspace memory. */
  memcpy((void *)rx_pkt_userspace, &(rx_pkt->data), rx_pkt->header.size);

  /* Mark the next packet as invalid. */
  *((uint32_t*)((uint8_t *)rx_pkt_userspace + rx_pkt->header.size)) = 0xFFFFFFFF;

  /* Phase 07.4.14 patch 0054 (A4): unified cleanup via helper.
   * Replaces the inline tx_pkt->header.size = 0; checksum = ... pair (and
   * the local tx_pkt declaration, no longer needed here) so
   * HAL_SPI_ErrorCallback below converges on the same site. */
  smart_md_publish_response_done();

  is_rx_buf_userspace_processed = false;

  /* 2026-10-22 (07.4.42.2 bench): SWAP AT COMPLETION. The old protocol re-armed the just-sent (now empty)
   * buffer here and swapped in new data from the main loop (dma_load(true) + HAL_SPI_Abort) whenever it
   * wanted to assert NIRQ — at an arbitrary moment relative to the master, which may be starting an
   * exchange right then: the abort tore down a clocked transfer, both directions of it were lost, and the
   * H7 loop stalled in the abort's FIFO-flush waits (heartbeat deltas: loop 103 iterations in 288 ms,
   * 0 SPI errors, 0 frames either way). Now the only place a DMA is armed is here, right after an exchange
   * ended, when the master is idle for >= X8H7_SPI_MIN_GAP_US (kernel side). The buffer armed is the one
   * that accumulated data since the previous completion; NIRQ (main.c) is a pure "data waiting" signal.
   * Cost: an H7->X8 item leaves one exchange later than before. */
  dma_load(true);

  /* 2026-10-22: if the buffer just armed holds data, tell the master NOW (from this ISR) instead of
   * waiting for the main loop (which may sit in WFI for up to a SysTick): NIRQ HIGH >= 5 us (patch 0056
   * minimum the i.MX8 edge detector needs), then LOW. The main loop's later HIGH/LOW re-signal is harmless. */
  {
    struct complete_packet *armed = (struct complete_packet *)p_tx_buf_transfer;
    if (armed->header.size > 0)
    {
      extern volatile uint32_t s_nirq_high_at_cycles;
      uint32_t const cycles_per_us = SystemCoreClock / 1000000u;
      uint32_t t0;
      set_nirq_high();
      t0 = DWT->CYCCNT;
      while ((DWT->CYCCNT - t0) < (5u * cycles_per_us)) { }
      set_nirq_low();
      s_nirq_high_at_cycles = 0;
    }
  }
}

/* 2026-10-22: SPI error-path prints go to the console UART (115200) from interrupt context — tens of
 * milliseconds with everything else stalled. Count instead; keep the text under DEBUG only. */
volatile uint32_t s_spi_error_callbacks = 0;
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  s_spi_error_callbacks++;
#ifdef DEBUG
  dbg_printf("HAL_SPI_ErrorCallback: spi error code = 0x%lX\n", HAL_SPI_GetError(hspi));
#else
  (void)hspi;
#endif

  /* 2026-10-22: do NOT zero the transfer buffer here (patch 0054 did). An SPI error means the
   * master clocked an exchange while our DMA was not armed (overrun): nothing of ours went out
   * and nothing of theirs is valid. Re-arm the SAME transfer buffer so its TX_COMPLETE / RX
   * frames go out on the next exchange instead of being lost (every loss stalls the kernel ring).
   * The RX side of this exchange is discarded by not touching is_rx_buf_userspace_processed. */
  dma_load(false);
  set_nirq_high();
}

void dma_handle_data()
{
  /* Enter critical section. */
  volatile uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1) ;

  while (!is_rx_buf_userspace_processed)
  {
    if (rx_pkt_userspace->header.peripheral != 0xFF &&
        rx_pkt_userspace->header.peripheral != 0x00)
    {
  #ifdef DEBUG
      {
        char dbg_msg[64] = {0};
        snprintf(dbg_msg,
                 sizeof(dbg_msg),
                 "dma_handle_data: %s op: %02X size: %d",
                 peripheral_to_string(rx_pkt_userspace->header.peripheral),
                 rx_pkt_userspace->header.opcode,
                 rx_pkt_userspace->header.size);

        char data_msg[64] = {0};
        uint16_t data_msg_len = 0;

        for (uint16_t i = 0; i < rx_pkt_userspace->header.size && i < sizeof(data_msg); i++)
          data_msg_len += snprintf(data_msg + data_msg_len, sizeof(data_msg) - data_msg_len, "%02X ", *((&rx_pkt_userspace->raw_data) + i));

        dbg_printf("%s data: %s\n", dbg_msg, data_msg);
      }
  #endif

      /* Invoke the registered callback for the selected peripheral. */
      int const rc = peripheral_invoke_callback(rx_pkt_userspace->header.peripheral,
                                                rx_pkt_userspace->header.opcode,
                                                (uint8_t *)(&(rx_pkt_userspace->raw_data)),
                                                rx_pkt_userspace->header.size);

      if (rc < 0) {
        dbg_printf("dma_handle_data: %s callback error: %d",
                  peripheral_to_string(rx_pkt_userspace->header.peripheral) , rc);
      }

      /* Advance to the next package. */
      rx_pkt_userspace = (struct subpacket *)((uint8_t *)rx_pkt_userspace + 4 /* sizeof(subpacket.header) */ + rx_pkt_userspace->header.size);
    }
    else
    {
      /* Mark the receive buffer as having been processed. */
      is_rx_buf_userspace_processed = true;
      /* Make sure that the RX packet processing pointer is pointing to the start of the receive buffer. */
      rx_pkt_userspace = (struct subpacket *)RX_Buffer_userspace;
      /* Enable IRQs sent by device again. */
      set_nirq_high();
      /* Phase 07.4.14 patch 0056 (A6): mark the NIRQ-high start cycle so the
       * main.c response-signaling block can enforce the minimum 5 µs HIGH
       * duration before pulling NIRQ low again. RESEARCH §1.4 falsifier
       * counter `s_nirq_high_pulse_short` increments in main.c when the
       * busy-wait actually fires. */
      extern volatile uint32_t s_nirq_high_at_cycles;
      s_nirq_high_at_cycles = DWT->CYCCNT;
    }
  }

  /* Leave critical section. */
  __set_PRIMASK(primask_bit);
}
