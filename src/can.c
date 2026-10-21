/* mbed Microcontroller Library
 * Copyright (c) 2006-2017 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "can.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_hsem.h"
#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "peripherals.h"
#include "error_handler.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define TARGET_STM32H7      1
#define CFG_HW_RCC_SEMID    3
#undef DUAL_CORE

/* Phase 07.1.2 — Transmitter Delay Compensation (TDC) for >=2 Mbit/s data phase.
 * Without TDC, the FDCAN controller's loop delay (TX pin -> own RX pin, used
 * for stuff-bit checking during transmission) exceeds the data-phase bit time,
 * causing the controller to flag its own TX as a stuff-bit error and retry
 * forever (H7-T5 TX_COMPLETE IRQ never fires).
 * TDCO derived per ST AN5348: TDCO ~= (DataTimeSeg1+1)/2 + (DataTimeSeg2+1)/2.
 * For our config (DataTimeSeg1=14, DataTimeSeg2=5): TDCO = 15/2 + 6/2 = 10.
 * TDCF=0 = no lower-window filter (safe default per RM0433).
 * Tuning: if TDCO=10 doesn't fire H7-T5, sweep 8..16 (HAL bounds 0..127). */
#define X8H7_FDCAN_TDCO     10
#define X8H7_FDCAN_TDCF     0

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FDCAN_HandleTypeDef fdcan_1;
FDCAN_HandleTypeDef fdcan_2;

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_FDCAN_CLK_ENABLE();
  HAL_RCC_FDCAN_CLK_ENABLED++;

  if (hfdcan->Instance == FDCAN1)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /*
     * FDCAN1 GPIO Configuration
     * PD1     ------> FDCAN1_TX
     * PD0     ------> FDCAN1_RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
  else if (hfdcan->Instance == FDCAN2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*
     * FDCAN2 GPIO Configuration
     * PB5     ------> FDCAN2_RX
     * PB6     ------> FDCAN2_TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  HAL_RCC_FDCAN_CLK_ENABLED--;
  if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
    __HAL_RCC_FDCAN_CLK_DISABLE();
  }

  if (hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_1 | GPIO_PIN_0);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
  else if (hfdcan->Instance == FDCAN2)
  {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  }
}

int can_internal_init(FDCAN_HandleTypeDef * handle)
{
  if (HAL_FDCAN_Init(handle) != HAL_OK)
    Error_Handler("HAL_FDCAN_Init Error_Handler\n");

  if (can_filter(handle, 0, 0, 0, false) == 0)
    Error_Handler("can_filter Error_Handler\n");

  if (can_filter(handle, 0, 0, 0, true) == 0)
    Error_Handler("can_filter Error_Handler\n");

  if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    Error_Handler("HAL_FDCAN_ConfigGlobalFilter Error_Handler\n");

  for (uint32_t txb_idx = 0; txb_idx < handle->Init.TxFifoQueueElmtsNbr; txb_idx++)
  {
    uint32_t const txb_bitmask = (1 << txb_idx);
    if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_TX_COMPLETE, txb_bitmask) != HAL_OK)
      Error_Handler("HAL_FDCAN_ActivateNotification(FDCAN_IT_TX_COMPLETE) Error_Handler\n");
    if (HAL_FDCAN_ActivateNotification(handle, FDCAN_FLAG_TX_ABORT_COMPLETE, txb_bitmask) != HAL_OK)
      Error_Handler("HAL_FDCAN_ActivateNotification(FDCAN_FLAG_TX_ABORT_COMPLETE) Error_Handler\n");
  }
  if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    Error_Handler("HAL_FDCAN_ActivateNotification(FDCAN_IT_TX_FIFO_EMPTY) Error_Handler\n");

  /* Phase 07.1.5 Task 2 (Tier 1A bridge cadence fix):
   * Add FDCAN_IT_RX_FIFO0_NEW_MESSAGE so the RX ISR fires on the very first
   * frame, not only after watermark=2 or FIFO_FULL. The error-handler string
   * already mentioned NEW_MESSAGE; the activation mask was missing it. This
   * is the secondary half of the cadence fix (paired with the 10→0 ms throttle
   * drop in can_handler.c) — together they make RX delivery truly event-driven.
   * Per Plan 07.1.5-01 INVESTIGATION-NOTES.md §"Proposed patch — Change 2". */
  if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_WATERMARK, 0) != HAL_OK)
    Error_Handler("HAL_FDCAN_ActivateNotification(FDCAN_IT_RX_FIFO0_NEW_MESSAGE) Error_Handler\n");

  /* Phase 07.4.42.2 (regenerated stm32h7-fw 0091 — 7.4.42.1 T3/T4 intent): arm the FDCAN error-state
   * notifications so HAL_FDCAN_ErrorStatusCallback (can_handler.c) reports bus-off / error-passive /
   * error-warning transitions (entry AND exit) with TEC/REC to the kernel. Low-rate; both FDCANs. The H7
   * never self-restarts: recovery is kernel-driven (can_bus_off → restart-ms → OC_DEINIT + OC_INIT). */
  if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0) != HAL_OK)
    Error_Handler("HAL_FDCAN_ActivateNotification(FDCAN_IT_BUS_OFF|ERROR_PASSIVE|ERROR_WARNING) Error_Handler\n");

  if (HAL_FDCAN_ConfigFifoWatermark(handle, FDCAN_CFG_RX_FIFO0, 2) != HAL_OK)
    Error_Handler("HAL_FDCAN_ConfigFifoWatermark(...) Error_Handler\n");

  /* Phase 07.1.2 — TDC must be configured AFTER HAL_FDCAN_Init (peripheral
   * initialized) but BEFORE HAL_FDCAN_Start (controller enters NORMAL mode).
   * Applies to both FDCAN1 + FDCAN2 by code symmetry (can_internal_init runs
   * once per peripheral). Harmless at sub-2-Mbit data phases per ST AN5348. */
  if (HAL_FDCAN_ConfigTxDelayCompensation(handle, X8H7_FDCAN_TDCO, X8H7_FDCAN_TDCF) != HAL_OK)
    Error_Handler("HAL_FDCAN_ConfigTxDelayCompensation Error_Handler\n");

  if (HAL_FDCAN_EnableTxDelayCompensation(handle) != HAL_OK)
    Error_Handler("HAL_FDCAN_EnableTxDelayCompensation Error_Handler\n");

  if (HAL_FDCAN_Start(handle) != HAL_OK)
    Error_Handler("HAL_FDCAN_Start Error_Handler\n");

  /* Phase 06.2 Plan 03c cycle 4 — H7-T8 trace. Snapshot four post-init
   * fields so the next bench cycle can confirm Mode=NORMAL(0),
   * AutoRetransmission=ENABLE(1), TxFifoQueueElmtsNbr=32, and that the
   * patch-0010 data-bittiming clamp took effect (DataTimeSeg1>=1).
   * Only emit for fdcan_1 to avoid double-noise; fdcan_2 init path is
   * structurally identical. */
  if (handle == &fdcan_1)
  {
    extern int enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void * data);
    uint8_t trace_buf[5] = {
      8,
      (uint8_t)(handle->Init.Mode & 0xFF),
      (uint8_t)(handle->Init.AutoRetransmission & 0xFF),
      (uint8_t)(handle->Init.TxFifoQueueElmtsNbr & 0xFF),
      (uint8_t)(handle->Init.DataTimeSeg1 & 0xFF),
    };
    /* PERIPH_H7=0x09, OPCODE_TRACE=0x79 — same channel as H7-T1..T7. */
    enqueue_packet(0x09, 0x79, sizeof(trace_buf), trace_buf);
  }

  /* Phase 06.2 Plan 03c cycle 5 — H7-T11 trace. Compute the runtime
   * total message-RAM bytes used by this peripheral and report whether
   * we fit inside the 1280 B per-peripheral budget (RM0399 §56.4.5).
   * Element sizes are hardcoded for FDCAN_DATA_BYTES_64 (=64 B data +
   * 8 B header = 72 B per FIFO/buffer element). Filters: Std=4 B,
   * Ext=8 B. TxEvent=8 B. */
  if (handle == &fdcan_1)
  {
    extern int enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void * data);
    uint32_t const total = handle->Init.StdFiltersNbr        * 4u
                         + handle->Init.ExtFiltersNbr        * 8u
                         + handle->Init.RxFifo0ElmtsNbr      * (8u + 64u)
                         + handle->Init.RxFifo1ElmtsNbr      * (8u + 64u)
                         + handle->Init.RxBuffersNbr         * (8u + 64u)
                         + handle->Init.TxEventsNbr          * 8u
                         + handle->Init.TxFifoQueueElmtsNbr  * (8u + 64u);
    uint8_t const budget_ok = (total <= 1280u) ? 1u : 0u;
    uint8_t trace_buf[5] = { 11, (uint8_t)(total & 0xFF), (uint8_t)((total >> 8) & 0xFF),
                             (uint8_t)((total >> 16) & 0xFF), budget_ok };
    enqueue_packet(0x09, 0x79, sizeof(trace_buf), trace_buf);
  }

  return 1;
}

void can_init(FDCAN_HandleTypeDef * handle, CANName peripheral,
              uint32_t const baud_rate_prescaler, uint32_t const time_segment_1,
              uint32_t const time_segment_2, uint32_t const sync_jump_width,
              uint32_t const data_baud_rate_prescaler, uint32_t const data_time_segment_1,
              uint32_t const data_time_segment_2, uint32_t const data_sync_jump_width)
{
    // Default values
    handle->Instance = (FDCAN_GlobalTypeDef *)peripheral;

    /* WARNING (Phase 06.5 D-08 fallback to Option A): FrameFormat is unconditional
     * FD-BRS. Acceptable for the controlled bench. MUST NOT ship to a mixed-vendor
     * bus where classic-only legacy nodes may bus-off on the first FD frame.
     * Reverse this commit + apply the Option B wire-format-extension patch
     * (deferred to a future hardening phase) before deploying on shared/external
     * CAN buses. See patches/stm32h7-fw/README.md "Phase 06.5 — FrameFormat
     * policy (Option A)" section for the deployment-risk audit trail. */
    handle->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    handle->Init.Mode = FDCAN_MODE_NORMAL;
    handle->Init.AutoRetransmission = ENABLE;
    handle->Init.TransmitPause = DISABLE;
    handle->Init.ProtocolException = ENABLE;
    handle->Init.NominalPrescaler = baud_rate_prescaler;
    handle->Init.NominalTimeSeg1 = time_segment_1;
    handle->Init.NominalTimeSeg2 = time_segment_2;
    handle->Init.NominalSyncJumpWidth = sync_jump_width;
    /* Phase 06.2 Plan 03c cycle 4 — clamp data-phase bittiming to the
     * upstream-baseline 0x1 if the kernel did not compute them (i.e.,
     * `ip link set canX type can bitrate N` without `dbitrate M fd on`).
     * USE_FULL_ASSERT is disabled, so passing 0 here would silently
     * leave DBTP DTSEG1/DTSEG2 = 0, putting the FDCAN in a state where
     * TX FIFO accepts but the peripheral never arbitrates. */
    handle->Init.DataPrescaler     = data_baud_rate_prescaler ? data_baud_rate_prescaler : 0x1;
    handle->Init.DataSyncJumpWidth = data_sync_jump_width     ? data_sync_jump_width     : 0x1;
    handle->Init.DataTimeSeg1      = data_time_segment_1      ? data_time_segment_1      : 0x1;
    handle->Init.DataTimeSeg2      = data_time_segment_2      ? data_time_segment_2      : 0x1;

    /* Message RAM offset is only supported in STM32H7 platforms of supported FDCAN platforms */
    /* Phase 06.5 D-05 — separate FDCAN1 (offset 0, 160 words) from FDCAN2
     * (offset 160, 160 words) in the 640-word SRAMCAN region. FDCAN2
     * footprint mirrors FDCAN1's reduced layout (Plan 03c cycle 5).
     * MessageRAMOffset is in 32-bit-word units per RM0399 §56.4.5 +
     * HAL_FDCAN_CalculateRamBlockAddresses. Total: 320 words = 1280 B = 50% of budget. */
    handle->Init.MessageRAMOffset = (handle == &fdcan_2) ? 160u : 0u;

    /* The number of Standard and Extended ID filters are initialized to the maximum possible extent
     * for STM32H7 platforms
     */
    handle->Init.StdFiltersNbr = 8;  // Plan 03c cycle 5 — reduced from 128 (RAM budget); only idx 0 used at init
    handle->Init.ExtFiltersNbr = 4;  // Plan 03c cycle 5 — reduced from 64 (RAM budget); only idx 0 used at init

    /* Phase 07.4.42.1 T2 — FDCAN2 FIFO depth + element-size hardening.
     * FDCAN2 (can1, motor bus) uses 32-byte elements (FDCAN_DATA_BYTES_32 =
     * 8 B header + 32 B data = 40 B/element).  No frame on can1 exceeds 24 B
     * (motor 0xA0 response = 24 B; SET_TARGETS/WRITE_REGISTER writes <= 12 B),
     * so 32-byte elements are sufficient and roughly halve per-element RAM cost.
     *
     * RAM math (FDCAN2 only, 32-byte elements):
     *   Std filters:  8  x 4 B       =  32 B
     *   Ext filters:  4  x 8 B       =  32 B
     *   RxFIFO0:     16  x 40 B      = 640 B   (4 -> 16-deep; absorbs 6-motor burst)
     *   TxFIFO:       8  x 40 B      = 320 B   (4 -> 8-deep; >= X8H7_CAN_BATCH_MAX=6)
     *   Total:                          1024 B  <= 1280 B (256 B headroom) OK
     *
     * FDCAN1 (cs8h7-can / RPMsg bus) keeps FDCAN_DATA_BYTES_64 because its max
     * FD frame size has not been independently verified to be <= 32 B.
     * Conditionalize both depth and element-size on handle == &fdcan_2 (Pitfall 2). */
    if (handle == &fdcan_2) {
      /* FDCAN2: motor bus -- 32-byte elements, deep FIFOs. */
      handle->Init.RxFifo0ElmtsNbr  = 16;                    /* 16 x 40 B = 640 B; absorbs 6-motor simultaneous 0xA0 burst */
      handle->Init.RxFifo0ElmtSize  = FDCAN_DATA_BYTES_32;
      handle->Init.RxFifo1ElmtsNbr  =  0;
      handle->Init.RxFifo1ElmtSize  = FDCAN_DATA_BYTES_32;
      handle->Init.RxBuffersNbr     =  0;
      handle->Init.RxBufferSize     = FDCAN_DATA_BYTES_32;
      handle->Init.TxEventsNbr         = 0;
      handle->Init.TxBuffersNbr        =  0;
      handle->Init.TxFifoQueueElmtsNbr = 8;                  /* 8 x 40 B = 320 B; >= X8H7_CAN_BATCH_MAX=6 */
      handle->Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;
      handle->Init.TxElmtSize          = FDCAN_DATA_BYTES_32;
    } else {
      /* FDCAN1: cs8h7-can / RPMsg bus -- keep 64-byte elements (Plan 03c cycle 5 baseline). */
      handle->Init.RxFifo0ElmtsNbr  =  4;                    /* 4 x 72 B = 288 B */
      handle->Init.RxFifo0ElmtSize  = FDCAN_DATA_BYTES_64;
      handle->Init.RxFifo1ElmtsNbr  =  0;
      handle->Init.RxFifo1ElmtSize  = FDCAN_DATA_BYTES_64;
      handle->Init.RxBuffersNbr     =  0;
      handle->Init.RxBufferSize     = FDCAN_DATA_BYTES_64;
      handle->Init.TxEventsNbr         = 0;
      handle->Init.TxBuffersNbr        =  0;
      handle->Init.TxFifoQueueElmtsNbr = 4;                  /* 4 x 72 B = 288 B */
      handle->Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;
      handle->Init.TxElmtSize          = FDCAN_DATA_BYTES_64;
    }

    can_internal_init(handle);
}

void can_deinit(FDCAN_HandleTypeDef * handle)
{
  HAL_FDCAN_Stop(handle);
  HAL_FDCAN_DeInit(handle);
}

int can_set_bittiming(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width)
{
  if (HAL_FDCAN_Stop(handle) != HAL_OK)
    Error_Handler("HAL_FDCAN_Stop Error_Handler\n");

  handle->Init.NominalPrescaler     = baud_rate_prescaler;
  handle->Init.NominalTimeSeg1      = time_segment_1;
  handle->Init.NominalTimeSeg2      = time_segment_2;
  handle->Init.NominalSyncJumpWidth = sync_jump_width;

  return can_internal_init(handle);
}

int can_filter(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask, bool const is_extended_id)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.IdType = is_extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = filter_index;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = is_extended_id ? (id & CAN_EFF_MASK) : (id & CAN_SFF_MASK);
  sFilterConfig.FilterID2 = is_extended_id ? (mask & CAN_EFF_MASK) : (mask & CAN_SFF_MASK);

  if (HAL_FDCAN_ConfigFilter(handle, &sFilterConfig) != HAL_OK) {
    return 0;
  }

  return 1;
}

uint32_t can_tx_fifo_available(FDCAN_HandleTypeDef * handle)
{
  return HAL_FDCAN_GetTxFifoFreeLevel(handle);
}

uint32_t can_rx_fifo_available(FDCAN_HandleTypeDef * handle, uint32_t const rx_fifo)
{
  return HAL_FDCAN_GetRxFifoFillLevel(handle, rx_fifo);
}

int can_write(FDCAN_HandleTypeDef * handle, uint32_t const id, uint8_t const len, uint8_t const * data, uint8_t const flags)
{
  FDCAN_TxHeaderTypeDef TxHeader = {0};

  if (id & CAN_EFF_FLAG)
  {
    TxHeader.IdType     = FDCAN_EXTENDED_ID;
    TxHeader.Identifier = id & CAN_EFF_MASK;
  }
  else
  {
    TxHeader.IdType     = FDCAN_STANDARD_ID;
    TxHeader.Identifier = id & CAN_SFF_MASK;
  }

    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    switch (len)
    {
      default:
      case 0:  TxHeader.DataLength = FDCAN_DLC_BYTES_0;  break;
      case 1:  TxHeader.DataLength = FDCAN_DLC_BYTES_1;  break;
      case 2:  TxHeader.DataLength = FDCAN_DLC_BYTES_2;  break;
      case 3:  TxHeader.DataLength = FDCAN_DLC_BYTES_3;  break;
      case 4:  TxHeader.DataLength = FDCAN_DLC_BYTES_4;  break;
      case 5:  TxHeader.DataLength = FDCAN_DLC_BYTES_5;  break;
      case 6:  TxHeader.DataLength = FDCAN_DLC_BYTES_6;  break;
      case 7:  TxHeader.DataLength = FDCAN_DLC_BYTES_7;  break;
      case 8:  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  break;
      case 12: TxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
      case 16: TxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
      case 20: TxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
      case 24: TxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
      case 32: TxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
      case 48: TxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
      case 64: TxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
    }
    TxHeader.ErrorStateIndicator = (flags & X8H7_CAN_FLAG_ESI)
                                   ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = (flags & X8H7_CAN_FLAG_BRS)
                                   ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    TxHeader.FDFormat            = (flags & X8H7_CAN_FLAG_FD)
                                   ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &TxHeader, (uint8_t *)data) != HAL_OK)
    {
      uint32_t const err_code = HAL_FDCAN_GetError(handle);
      dbg_printf("HAL_FDCAN_AddMessageToTxFifoQ failed with %ld\n", err_code);
      return -err_code;
    }
    return 0;
}

int can_read(FDCAN_HandleTypeDef * handle, uint32_t * id, uint8_t * len, uint8_t * data, uint8_t * flags_out)
{
  static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

  if (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) == 0)
    return 0; // No message arrived

  FDCAN_RxHeaderTypeDef RxHeader = {0};
  uint8_t RxData[64] = {0};
  if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler("HAL_FDCAN_GetRxMessage Error_Handler\n"); // Should not occur as previous HAL_FDCAN_GetRxFifoFillLevel call reported some data
    return 0;
  }

  if (RxHeader.IdType == FDCAN_EXTENDED_ID)
    *id = CAN_EFF_FLAG | (RxHeader.Identifier & CAN_EFF_MASK);
  else
    *id =                (RxHeader.Identifier & CAN_SFF_MASK);

  if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
    *id |= CAN_RTR_FLAG;

  *len = DLCtoBytes[RxHeader.DataLength >> 16];
  /* Guard uses X8H7_CAN_FRAME_MAX_DATA_LEN (= 64 after patch 0003).
   * Do NOT replace with a hardcoded literal -- the macro expresses intent. */
  if (*len > X8H7_CAN_FRAME_MAX_DATA_LEN)
    *len = X8H7_CAN_FRAME_MAX_DATA_LEN;

  *flags_out = 0;
  if (RxHeader.FDFormat == FDCAN_FD_CAN)
    *flags_out |= X8H7_CAN_FLAG_FD;
  if (RxHeader.BitRateSwitch == FDCAN_BRS_ON)
    *flags_out |= X8H7_CAN_FLAG_BRS;
  if (RxHeader.ErrorStateIndicator == FDCAN_ESI_PASSIVE)
    *flags_out |= X8H7_CAN_FLAG_ESI;

  memcpy(data, RxData, *len);

  return 1;
}

unsigned char can_rdError_Handler(FDCAN_HandleTypeDef * handle)
{
  FDCAN_ErrorCountersTypeDef ErrorCounters;
  HAL_FDCAN_GetErrorCounters(handle, &ErrorCounters);
  return (unsigned char)ErrorCounters.RxErrorCnt;
}

unsigned char can_tdError_Handler(FDCAN_HandleTypeDef * handle)
{
  FDCAN_ErrorCountersTypeDef ErrorCounters;
  HAL_FDCAN_GetErrorCounters(handle, &ErrorCounters);
  return (unsigned char)ErrorCounters.TxErrorCnt;
}
