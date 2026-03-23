#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32f4xx.h"
#include "board.h"

#define R_TOP_OHM      100000.0f  /* 电池正极到ADC点的上侧电阻 R1 */
#define R_BOTTOM_OHM   100000.0f  /* ADC点到地的下侧电阻 R2 */

#define VREF_V         3.3f
#define BATTERY_ADC_MAX        4095.0f
// #define VOLTAGE_DIVIDER_RATIO  ((R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM)
#define VOLTAGE_DIVIDER_RATIO  2.0f/30.0f
#define ADC_RAW_TO_MV_FACTOR  ((VREF_V * 1000.0f * VOLTAGE_DIVIDER_RATIO) / BATTERY_ADC_MAX)

#define BATTERY_FULL    4200  // 4.2V
#define BATTERY_EMPTY   3300  // 3.3V

#define PWR_STATE_PREPARE   0
#define PWR_STATE_READ      1
#define PWR_STATE_WAIT      2
#define PWR_STATE_DISABLE   3

extern __IO uint16_t pwr_adc_raw;
extern __IO uint16_t pwr_state;

void Battery_ADC_Init(void);
void Battery_Measure_Reset(void);
void Battery_Measure_Step(void);
uint8_t Battery_GetPercentage(void);
uint32_t Battery_GetVoltage(void);

#endif /* __BATTERY_H */