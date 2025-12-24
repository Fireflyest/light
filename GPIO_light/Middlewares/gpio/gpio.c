#include "gpio.h"

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable peripheral clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // For I2C pins on GPIOB
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // For UART pins on GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   // Enable I2C1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable USART1 clock




    /* Configure C13 pin(PC13) in output function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);




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
    



    /* Configure USART1 pins: TX (PB6) and RX (PB7) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // Push-pull for UART
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // Pull-up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB6 and PB7 to USART1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);




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

void UART_Config(int baudrate) {
    USART_InitTypeDef USART_InitStructure;
    
    // 先禁用USART1再配置
    USART_Cmd(USART1, DISABLE);

    /* USART1 configuration for Bluetooth module */
    USART_InitStructure.USART_BaudRate = baudrate;                  // Standard baud rate for most BT modules
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8 data bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1 stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No;         // No parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Enable both transmit and receive
    
    /* Initialize USART1 */
    USART_Init(USART1, &USART_InitStructure);

    USART_ClearFlag(USART1, USART_FLAG_TC | USART_FLAG_RXNE);
    
    // 配置 USART 中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);

    // 使能 USART 接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

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