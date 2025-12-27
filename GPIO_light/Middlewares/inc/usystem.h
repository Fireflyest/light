# ifndef __USYSTEM_H
# define __USYSTEM_H

# include "stm32f4xx.h"

# include "usart1.h"
# include "dma.h"
# include "key.h"
# include "led.h"
# include "pwm.h"


extern __IO uint32_t uwTimingDelay;


void delay_ms(__IO uint32_t nTime);

void Init_USystem();
void Init_USART(uint16_t baudrate);
void Init_PWM(uint16_t period, uint16_t prescaler);

# endif /* __USYSTEM_H */