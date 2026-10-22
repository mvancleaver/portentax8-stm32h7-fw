/* crash_diag.c — see crash_diag.h. 2026-10-22 (07.4.42.2 bench). */
#include "crash_diag.h"

#define CRASH_MAGIC 0xFA17C0DEu

static uint16_t            s_reset_cause;
static uint16_t            s_boot_count;
static uint8_t             s_last_stage;
static struct crash_record s_last_fault;

static void bkp_enable(void)
{
  RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
  (void)RCC->APB4ENR;
  PWR->CR1 |= PWR_CR1_DBP;
  while ((PWR->CR1 & PWR_CR1_DBP) == 0u) { }
}

void crash_diag_boot_capture(void)
{
  s_reset_cause = (uint16_t)(RCC->RSR >> 16);
  RCC->RSR |= RCC_RSR_RMVF;
  bkp_enable();
  s_boot_count = (uint16_t)(RTC->BKP7R + 1u);
  RTC->BKP7R   = s_boot_count;
  s_last_stage = (uint8_t)RTC->BKP8R;
  if (RTC->BKP0R == CRASH_MAGIC)
  {
    s_last_fault.kind = (uint8_t)RTC->BKP6R;
    s_last_fault.cfsr = RTC->BKP1R;
    s_last_fault.hfsr = RTC->BKP2R;
    s_last_fault.pc   = RTC->BKP3R;
    s_last_fault.lr   = RTC->BKP4R;
    s_last_fault.addr = RTC->BKP5R;
    RTC->BKP0R = 0u;
  }
  RTC->BKP8R = 0u;
}

void crash_diag_fault(uint32_t *sp, uint32_t kind)
{
  bkp_enable();
  RTC->BKP6R = kind;
  RTC->BKP1R = SCB->CFSR;
  RTC->BKP2R = SCB->HFSR;
  RTC->BKP5R = (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) ? SCB->BFAR
             : (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) ? SCB->MMFAR : 0u;
  RTC->BKP3R = 0u;
  RTC->BKP4R = 0u;
  RTC->BKP0R = CRASH_MAGIC;   /* record valid from here even if the stack reads below fault again */
  RTC->BKP4R = sp[5];         /* stacked LR */
  RTC->BKP3R = sp[6];         /* stacked PC */
  NVIC_SystemReset();
  for (;;) { }
}

uint16_t crash_diag_reset_cause(void) { return s_reset_cause; }
uint16_t crash_diag_boot_count(void)  { return s_boot_count; }
uint8_t  crash_diag_last_stage(void)  { return s_last_stage; }
struct crash_record const *crash_diag_last_fault(void) { return &s_last_fault; }
