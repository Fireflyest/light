#include "gpio.h"

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable peripheral clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // For I2C pins on GPIOB
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // For UART pins on GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   // Enable I2C1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable USART1 clock



    /* Configure I2C1 pins: SCL (PB8) and SDA (PB9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;       // Open Drain for I2C
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         // Pull-up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB8 and PB9 to I2C1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    



    /* Configure TIM3 pins: CH3 (PB0) and CH4 (PB1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // Push-pull
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // Pull-up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB0 and PB1 to TIM3 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

    /* Configure TIM3 pins: CH1 (PA6) and CH2 (PA7) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // Push-pull
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // Pull-up resistors
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Connect PA6 and PA7 to TIM3 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
}

void I2C_Config(void) {
    I2C_InitTypeDef I2C_InitStructure;
    
    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;           // Own address (not used in master mode)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;          // 400 KHz (Fast mode)
    
    /* Initialize I2C peripheral */
    I2C_Init(I2C1, &I2C_InitStructure);

    /* Enable I2C */
    I2C_Cmd(I2C1, ENABLE);
}


/**
 * Freq = Timer_Clock / ((Prescaler + 1) * (Period + 1))
 * Duty Cycle (%) = (Pulse / (Period + 1)) * 100
 * Resolution = Timer_Clock / (Prescaler + 1)
 * 
 * @brief  Configure TIM3 for PWM output on 4 channels
 * @param  period: PWM period (ARR value)
 * @param  prescaler: Timer prescaler value
 * @retval None
 */
void PWM_TIM_Config(uint16_t period, uint16_t prescaler) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 使能 TIM3 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* 基本定时器配置 */
    TIM_TimeBaseStructure.TIM_Period = period - 1;           // 自动重载值
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;     // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    /* 通用配置对象 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        // PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性高
    
    /* 通道 1 配置 (PA6) */
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为 0
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* 通道 2 配置 (PA7) */
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为 0
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* 通道 3 配置 (PB0) */
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为 0
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* 通道 4 配置 (PB1) */
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为 0
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* 使能自动重装载寄存器的预装载 */
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    /* 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);
}