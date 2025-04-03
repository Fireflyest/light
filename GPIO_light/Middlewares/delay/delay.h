#ifndef __delay_H
#define __delay_H 


#include "stm32f4xx.h"

void Delay_Decrement(void);

void delay_ms(__IO uint32_t nTime);

void delay_us(__IO uint32_t nTime);

#endif
