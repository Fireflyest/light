#ifndef __PWR_H
#define __PWR_H

#include "stm32f4xx.h"

#define R_TOP_OHM      10000.0f  /* 电池正极到ADC点的上侧电阻 R1 */
#define R_BOTTOM_OHM   10000.0f  /* ADC点到地的下侧电阻 R2 */

#define ADC_MAX        4095.0f
#define VREF_V         3.3f

#define BATTERY_FULL    4200  // 4.2V
#define BATTERY_EMPTY   3300  // 3.3V

#define PWR_STATE_PREPARE   0
#define PWR_STATE_READ      1
#define PWR_STATE_WAIT      2
#define PWR_STATE_DISABLE   3

extern __IO uint16_t pwr_adc_raw;
extern __IO uint16_t pwr_state;

void Init_PWR(void);
void PWR_Handle(void);
uint8_t PWR_GetPercentage(void);

#endif /* __PWR_H */