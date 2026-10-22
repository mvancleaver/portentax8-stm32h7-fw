/* crash_diag.h — 2026-10-22 (07.4.42.2 bench): why did the H7 reboot?
 * Reset cause (RCC->RSR) captured at boot; fault registers + stacked PC/LR written by the fault handlers
 * into RTC backup registers (survive every reset but power-on) and read back at boot; a main-loop stage
 * breadcrumb (RTC->BKP8R) for IWDG-only resets. All of it goes out in the FDCAN2 heartbeat v3 tail. */
#ifndef CRASH_DIAG_H_
#define CRASH_DIAG_H_
#include <stdint.h>

struct crash_record {
  uint8_t  kind;     /* 0 none, 1 HardFault, 2 MemManage, 3 BusFault, 4 UsageFault, 5 NMI */
  uint32_t cfsr, hfsr, pc, lr, addr;
};

void crash_diag_boot_capture(void);                 /* first thing in main() */
void crash_diag_fault(uint32_t *sp, uint32_t kind); /* called by the naked fault handlers; never returns */
static inline void crash_diag_stage(uint8_t s);     /* main-loop breadcrumb */

uint16_t crash_diag_reset_cause(void);              /* RCC->RSR bits 16..31 */
uint16_t crash_diag_boot_count(void);
uint8_t  crash_diag_last_stage(void);
struct crash_record const *crash_diag_last_fault(void);

#include "stm32h7xx_hal.h"
static inline void crash_diag_stage(uint8_t s) { RTC->BKP8R = s; }
#endif
