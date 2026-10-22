/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "crash_diag.h"
#include "smart_md_handler.h"  /* Phase 7.4 — htim7_smart_md handle */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef fdcan_1;
extern FDCAN_HandleTypeDef fdcan_2;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
__attribute__((naked)) void NMI_Handler(void) {
  /* 2026-10-22 crash_diag: select the faulting stack, record registers, reset (no 2 s IWDG wait). */
  __asm volatile(
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "mov r1, #5          \n"
    "b crash_diag_fault  \n");
}

/**
 * @brief This function handles Hard fault interrupt.
 */
__attribute__((naked)) void HardFault_Handler(void) {
  /* 2026-10-22 crash_diag: select the faulting stack, record registers, reset (no 2 s IWDG wait). */
  __asm volatile(
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "mov r1, #1          \n"
    "b crash_diag_fault  \n");
}

/**
 * @brief This function handles Memory management fault.
 */
__attribute__((naked)) void MemManage_Handler(void) {
  /* 2026-10-22 crash_diag: select the faulting stack, record registers, reset (no 2 s IWDG wait). */
  __asm volatile(
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "mov r1, #2          \n"
    "b crash_diag_fault  \n");
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
__attribute__((naked)) void BusFault_Handler(void) {
  /* 2026-10-22 crash_diag: select the faulting stack, record registers, reset (no 2 s IWDG wait). */
  __asm volatile(
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "mov r1, #3          \n"
    "b crash_diag_fault  \n");
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
__attribute__((naked)) void UsageFault_Handler(void) {
  /* 2026-10-22 crash_diag: select the faulting stack, record registers, reset (no 2 s IWDG wait). */
  __asm volatile(
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "mov r1, #4          \n"
    "b crash_diag_fault  \n");
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
  HAL_IncTick();
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

/**
 * @brief This function handles FDCAN1 interrupt 0.
 */
void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&fdcan_1);
}

/**
 * @brief This function handles FDCAN2 interrupt 0.
 */
void FDCAN2_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&fdcan_2);
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void) {
  HAL_SPI_IRQHandler(&hspi3);
}

/**
 * @brief This function handles TIM7 global interrupt — Phase 7.4 smart-MD
 *        cache-poll IRQ at 1 kHz. The HAL TIM IRQ handler dispatches to
 *        HAL_TIM_PeriodElapsedCallback which is overridden in
 *        src/smart_md_handler.c to call smart_md_cache_refresh_tick().
 */
void TIM7_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim7_smart_md);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
