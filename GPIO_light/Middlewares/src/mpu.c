# include "mpu.h"

__IO uint8_t mpuReadDone;
uint8_t mpuDataBuffer[14];

void Init_MPU_Hardware(void) {
    // Enable I2C2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure I2C2 pins: SCL (PB10) and SDA (PB3) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;       // Open Drain for I2C
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         // Pull-up
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB10 and PB3 to I2C2 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF9_I2C2); 


    I2C_DeInit(I2C2);
    I2C_InitTypeDef I2C_InitStructure;
    /* I2C2 configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;           // Own address (not used in master mode)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;          // 400 KHz (Fast mode)
    /* Initialize I2C2 peripheral */
    I2C_Init(I2C2, &I2C_InitStructure);
    /* Enable I2C2 */
    I2C_Cmd(I2C2, ENABLE);
}

void Read_MPU_All(void) {
    mpuReadDone = 0;

    uint8_t reg = MPU6050_RA_ACCEL_XOUT_H;
    uint8_t timeout;
    for (timeout = 0xFFFF; timeout > 0 && I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY); timeout--);
    I2C_GenerateSTART(I2C2, ENABLE);
    for (timeout = 0xFFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); timeout--);
    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    for (timeout = 0xFFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); timeout--);
    I2C_SendData(I2C2, reg);
    for (timeout = 0xFFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED); timeout--);

    // 2. 重新起始，进入接收模式
    I2C_GenerateSTART(I2C2, ENABLE);
    for (timeout = 0xFFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); timeout--);
    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
    for (timeout = 0xFFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); timeout--);

    // 3. 启动DMA
    DMA_Cmd(DMA1_Stream2, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream2, 14);
    DMA_Cmd(DMA1_Stream2, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}