# ifndef __USYSTEM_H
# define __USYSTEM_H

# include "stm32f4xx.h"

# include "usart1.h"
# include "dma.h"
# include "key.h"
# include "led.h"
# include "pwm.h"
# include "oled.h"
# include "math3d.h"
# include "gfx.h"
# include "ui.h"
# include "mpu.h"


extern __IO uint16_t sysTick;
extern UI_Logger logWindow;

void delay_ms(__IO uint32_t nTime);

void Init_USystem();

void Init_Display();
void Init_USART(uint16_t baudrate);
void Init_PWM(uint16_t period, uint16_t prescaler);
void Init_MPU();
void Init_Widgets();

void System_Update_Task();

void Loop();

# endif /* __USYSTEM_H */