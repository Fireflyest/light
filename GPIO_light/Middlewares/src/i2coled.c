#include "i2coled.h"
#include "dma.h" // Assumes Init_DMA_For_I2C1_TX is defined here

#define OLED_ADDRESS 0x78

// Initialization Commands for SH1106 / SSD1306
static const uint8_t OLED_Init_Cmds[] = {
    0xAE, // Display Off
    0x20, 0x02, // Set Memory Addressing Mode to Page Mode (0x02) for SH1106 compatibility
    0xB0, // Set Page Start Address for Page Addressing Mode, 0-7
    0xC8, // Set COM Output Scan Direction
    0x00, // Set low column address
    0x10, // Set high column address
    0x40, // Set start line address
    0x81, 0xCF, // Set contrast control register
    0xA1, // Set segment re-map 0 to 127
    0xA6, // Set normal display
    0xA8, 0x3F, // Set multiplex ratio(1 to 64)
    0xA4, // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    0xD3, 0x00, // Set display offset
    0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
    0xD9, 0xF1, // Set pre-charge period
    0xDA, 0x12, // Set com pins hardware configuration
    0xDB, 0x40, // Set vcomh
    0x8D, 0x14, // Set Charge Pump enable/disable
    0xAF  // Turn on oled panel
};

// Internal I2C GPIO and Peripheral Initialization
static void I2C_Configuration(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
    /* Configure I2C1 pins: SCL (PB8) and SDA (PB9) */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;       // Open Drain for I2C
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         // Pull-up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB8 and PB9 to I2C1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    I2C_InitTypeDef I2C_InitStructure;
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

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void OLED_WriteCmd(uint8_t cmd) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, OLED_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, 0x00); // Co=0, D/C#=0 (Command)
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, cmd);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void OLED_WriteData(uint8_t data) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, OLED_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, 0x40); // Co=0, D/C#=1 (Data)
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void Init_OLED_Hardware(void) {
    I2C_Configuration();
    
    // Send Initialization Sequence
    for (int i = 0; i < sizeof(OLED_Init_Cmds); i++) {
        OLED_WriteCmd(OLED_Init_Cmds[i]);
    }
    
    // Enable I2C DMA Request feature
    I2C_DMACmd(I2C1, ENABLE);
}

void OLED_UpdatePage_DMA(uint8_t pageIdx, uint8_t* pBuffer) {
    // 1. Set Page and Column Address (SH1106 specific offset 0x02)
    OLED_WriteCmd(0xB0 + pageIdx);
    OLED_WriteCmd(0x02); // Lower Column Start
    OLED_WriteCmd(0x10); // Higher Column Start

    // 2. Prepare I2C for Data Stream
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    I2C_Send7bitAddress(I2C1, OLED_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Send Control Byte 0x40 (Data Stream follows)
    I2C_SendData(I2C1, 0x40);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // 3. Trigger DMA for 128 bytes of pixel data
    // Note: Init_DMA_For_I2C1_TX must be defined in dma.c
    Init_DMA_For_I2C1_TX(pBuffer, 128);
    
    // Enable DMA TC interrupt to send STOP (handled in stm32f4xx_it.c)
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

uint8_t OLED_IsDMABusy(void) {
    return (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
}