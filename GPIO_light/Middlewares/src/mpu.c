# include "mpu.h"

__IO uint8_t mpuReadDone;
uint8_t mpuDataBuffer[14];

void Init_MPU_Hardware(void) {
    #ifdef COMMUNICATION_TYPE_I2C
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
    #endif

    #ifdef COMMUNICATION_TYPE_SPI
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure SPI2 pins: SCK/SCL (PB13), MISO/AD0 (PB14), MOSI/SDA (PB15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    // CS as GPIO output (active low)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12); // drive NCS low to select SPI mode

    // SPI2 init (master, Mode 0)
    SPI_I2S_DeInit(SPI2);
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        // Mode 0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // adjust for speed
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
    #endif

    Write_MPU_Register(MPU_PWR_MGMT_1, 0x01);
    Write_MPU_Register(MPU_PWR_MGMT_2, 0x00);
    Write_MPU_Register(MPU_SMPLRT_DIV, 0x09);
    Write_MPU_Register(MPU_CONFIG, 0x06);
    Write_MPU_Register(MPU_GYRO_CONFIG, 0x18);
    Write_MPU_Register(MPU_ACCEL_CONFIG, 0x18);
}

#ifdef COMMUNICATION_TYPE_SPI
static inline void SPI_CS_Low(void)  { GPIO_ResetBits(GPIOB, GPIO_Pin_12); }
static inline void SPI_CS_High(void) { GPIO_SetBits(GPIOB, GPIO_Pin_12); }

static uint8_t spi_transfer_byte(uint8_t tx) {
    // wait TXE
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, tx);
    // wait RXNE
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}
#endif

void Write_MPU_Register(uint8_t reg, uint8_t data) {
    #ifdef COMMUNICATION_TYPE_I2C
    uint8_t timeout;
    for (timeout = 0xFF; timeout > 0 && I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY); timeout--);
    I2C_GenerateSTART(I2C2, ENABLE);
    for (timeout = 0xFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); timeout--);
    I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Transmitter);
    for (timeout = 0xFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); timeout--);
    I2C_SendData(I2C2, reg);
    for (timeout = 0xFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED); timeout--);
    I2C_SendData(I2C2, data);
    for (timeout = 0xFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED); timeout--);
    I2C_GenerateSTOP(I2C2, ENABLE);
    #endif

    #ifdef COMMUNICATION_TYPE_SPI
    SPI_CS_Low();
    // send register address (ensure MSB = 0 for write)
    (void)spi_transfer_byte((uint8_t)(reg & 0x7F));
    // send data
    (void)spi_transfer_byte(data);
    SPI_CS_High();
    #endif
}

void Read_MPU_All(void) {
    #ifdef COMMUNICATION_TYPE_I2C
    uint32_t timeout;
    for (timeout = 0x1FFFF; timeout > 0 && I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY); timeout--);
    I2C_GenerateSTART(I2C2, ENABLE);
    for (timeout = 0x1FFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); timeout--);
    I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Transmitter);
    for (timeout = 0x1FFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); timeout--);
    I2C_SendData(I2C2, MPU_ACCEL_XOUT_H);
    for (timeout = 0x1FFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED); timeout--);
    I2C_GenerateSTART(I2C2, ENABLE);
    for (timeout = 0x1FFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); timeout--);
    I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Receiver);
    for (timeout = 0x1FFFF; timeout > 0 && !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); timeout--);
    for (int i = 0; i < 14; ++i) {
        if (i == 13) {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            I2C_GenerateSTOP(I2C2, ENABLE);
        }
        for (timeout = 0x1FFFF; timeout > 0 && !(I2C2->SR1 & I2C_SR1_RXNE); timeout--);
        mpuDataBuffer[i] = I2C_ReceiveData(I2C2);
    }
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    #endif

    #ifdef COMMUNICATION_TYPE_SPI
    SPI_CS_Low();
    // send register address with Read bit (typically MSB=1 for MPU SPI read)
    (void)spi_transfer_byte((uint8_t)(MPU_ACCEL_XOUT_H | 0x80));
    for (int i = 0; i < 14; ++i) {
        mpuDataBuffer[i] = spi_transfer_byte(0xFF);
    }
    SPI_CS_High();
    #endif
}