# ifndef __PWM_H
# define __PWM_H

# include "stm32f4xx.h"

# define PWM_PERIOD     500    // PWM 周期
# define PWM_PRESCALER  84       // 84MHz 时钟下，分频为 84 得到 1MHz 的计数频率

# define PWM_MAX_DUTY   PWM_PERIOD          // 最大占空比
# define PWM_MIN_DUTY   0                   // 最小占空比
# define PWM_MAX_REAL   2000                // 硬件最大实际值
# define PWM_MIN_REAL   1000                // 硬件最小实际值


extern uint16_t pwmDutyBuffer[4];

void PWM_TIM_Init(uint16_t period, uint16_t prescaler);
uint8_t PWM_Map_Percent(uint16_t percent);

# endif /* __PWM_H */