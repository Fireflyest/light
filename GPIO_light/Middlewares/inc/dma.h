# ifndef __DMA_H
# define __DMA_H

#include "stm32f4xx.h"

void Init_DMA_For_USART1_RX(uint8_t* buffer, uint32_t bufferSize);
void Init_DMA_For_USART1_TX(uint8_t* buffer);

void Init_DMA_For_PWM_TIM3(uint16_t* buffer);

# endif /* __DMA_H */