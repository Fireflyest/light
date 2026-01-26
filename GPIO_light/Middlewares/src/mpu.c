# include "mpu.h"
# include "arm_math.h"
# include "math.h"

uint8_t mpuDataBuffer[14];
uint8_t magDataBuffer[7];
uint8_t bmpDataBuffer[6];

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp_calib_t;
static bmp_calib_t bmp_calib;
static int32_t t_fine;
float temperature;
float barometricPressure;
float altitude;

static void Init_IMU_GPIO(void);
static void Init_MPU_Hardware(void);
static void Init_BMP_Hardware(void);
static void Init_ICM_Hardware(void);
static void ICM_SelectBank(uint8_t bank);

static void Read_BMP_Calibration(void);
static void BMP_Compensate_T(int32_t adc_T);
static void BMP_Compensate_P(int32_t adc_P);
static void BMP_GetAltitude();

#ifdef COMMUNICATION_TYPE_SPI
static inline void MPU_SPI_CS_ON(void)  { GPIO_ResetBits(GPIOB, GPIO_Pin_12); }
static inline void MPU_SPI_CS_OFF(void) { GPIO_SetBits(GPIOB, GPIO_Pin_12); }
static inline void BMP_SPI_CS_ON(void)  { GPIO_ResetBits(GPIOA, GPIO_Pin_5); }
static inline void BMP_SPI_CS_OFF(void) { GPIO_SetBits(GPIOA, GPIO_Pin_5); }
static uint8_t spi_transfer_byte(uint8_t tx) {
    // wait TXE
    uint16_t timeout;
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET; timeout--);
    SPI_I2S_SendData(SPI2, tx);
    // wait RXNE
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET; timeout--);
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}
#endif // #ifdef COMMUNICATION_TYPE_SPI


void Init_IMU_Hardware(void) {
    Init_MPU_Hardware();
    Init_ICM_Hardware();
    Init_BMP_Hardware();
}

void Init_IMU_GPIO(void) {
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
    #endif // #ifdef COMMUNICATION_TYPE_I2C

    #ifdef COMMUNICATION_TYPE_SPI
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
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
    GPIO_ResetBits(GPIOB, GPIO_Pin_12); // spi mode

    #ifdef BMP280
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_5); // spi mode
    #endif // #ifdef BMP280

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
    #endif // #ifdef COMMUNICATION_TYPE_SPI
}

void Init_MPU_Hardware(void) {
    #if defined(MPU_6500) || defined(MPU_9250)
    Write_MPU_Register(MPU_PWR_MGMT_1, 0x01);
    Write_MPU_Register(MPU_PWR_MGMT_2, 0x00);
    Write_MPU_Register(MPU_SMPLRT_DIV, 0x09);
    Write_MPU_Register(MPU_CONFIG, 0x06);
    Write_MPU_Register(MPU_GYRO_CONFIG, 0x18);
    Write_MPU_Register(MPU_ACCEL_CONFIG, 0x18);
    #endif // #if defined(MPU_6500) || defined(MPU_9250
}

void Init_ICM_Hardware(void) {
    #ifdef ICM_20948
    ICM_SelectBank(0);
    
    // 2. 解除睡眠模式 (PWR_MGMT_1 = 0x6B)
    // 必须先把原本的 0x41 (睡眠位) 改为 0x01 (自动选择时钟源)
    Write_MPU_Register(ICM_PWR_MGMT_1, 0x01); // ICM-20948 的 PWR_MGMT_1 地址通常是 0x06
    Write_MPU_Register(ICM_PWR_MGMT_2, 0x00); // 启用所有引脚

    // 2. 开启 I2C Master 模式 (用于读取磁力计)
    // USER_CTRL 地址为 0x03，位 5 (I2C_MST_EN) 置 1
    Write_MPU_Register(0x03, 0x20); 

    ICM_SelectBank(3);
    Write_MPU_Register(0x01, 0x07); // I2C_MST_CTRL: 约 345kHz

    Write_MPU_Register(0x03, 0x0C); // I2C_SLV0_ADDR: Mag写模式地址 0x0C
    Write_MPU_Register(0x04, 0x31); // I2C_SLV0_REG: CNTL2 寄存器
    Write_MPU_Register(0x06, 0x08); // I2C_SLV0_DO: 写入 0x08 (100Hz 模式4)
    Write_MPU_Register(0x05, 0x81); // I2C_SLV0_CTRL: 使能, 写入 1 字节

    // 4. 配置磁力计 AK09916 自动读取 (Slave 0)
    // AK09916 的 I2C 地址是 0x0C，设置位 7 为 1 表示读
    Write_MPU_Register(0x03, 0x80 | 0x0C); // I2C_SLV0_ADDR
    Write_MPU_Register(0x04, 0x11);        // I2C_SLV0_REG: 从磁力计 0x11 (HXL) 开始读
    Write_MPU_Register(0x05, 0x89);        // I2C_SLV0_CTRL: 启用 Slave0，读取 9 个字节 (包含 ST2 状态)

    // 3. 切换到 Bank 2 配置量程
    ICM_SelectBank(2);
    Write_MPU_Register(ICM_GYRO_CONFIG_1, 0x19);   // ±2000 dps
    Write_MPU_Register(ICM_ACCEL_CONFIG, 0x11);      // ±16 g
    
    // 4. 切回 Bank 0 准备读数据
    ICM_SelectBank(0);
    #endif // #ifdef ICM_20948
}


void Init_BMP_Hardware(void) {
    #ifdef BMP280
    Write_BMP_Register(BMP_CTRL_MEAS, 0x27); // normal mode, temp and pressure oversampling x1
    Write_BMP_Register(BMP_CONFIG, 0xA0);    // standby 1000ms, filter off
    Read_BMP_Calibration();
    #endif //#ifdef BMP280
}

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
    #endif // #ifdef COMMUNICATION_TYPE_I2C

    #ifdef COMMUNICATION_TYPE_SPI
    MPU_SPI_CS_ON();
    // send register address (ensure MSB = 0 for write)
    (void)spi_transfer_byte((uint8_t)(reg & 0x7F));
    // send data
    (void)spi_transfer_byte(data);
    MPU_SPI_CS_OFF();
    #endif // #ifdef COMMUNICATION_TYPE_SPI
}

void Write_BMP_Register(uint8_t reg, uint8_t data) {
    #ifdef COMMUNICATION_TYPE_SPI
    BMP_SPI_CS_ON();
    // send register address (ensure MSB = 0 for write)
    (void)spi_transfer_byte((uint8_t)(reg & 0x7F));
    // send data
    (void)spi_transfer_byte(data);
    BMP_SPI_CS_OFF();
    #endif // #ifdef COMMUNICATION_TYPE_SPI
}


uint8_t Read_MPU_Register(uint8_t reg) {
    return 0;
}

uint8_t Read_BMP_Register(uint8_t reg) {
    uint8_t v;
    BMP_SPI_CS_ON();
    for (volatile int i = 0; i < 200; ++i); // short settle delay
    (void)spi_transfer_byte((uint8_t)(reg | 0x80)); // read flag
    v = spi_transfer_byte(0xFF);
    BMP_SPI_CS_OFF();
    return v;
}

void Read_IMU_All(void) {
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
    #endif // #ifdef COMMUNICATION_TYPE_I2C

    #ifdef COMMUNICATION_TYPE_SPI

    #ifdef MPU_6500

    MPU_SPI_CS_ON();
    // send register address with Read bit (typically MSB=1 for MPU SPI read)
    (void)spi_transfer_byte((uint8_t)(MPU_ACCEL_XOUT_H | 0x80));
    for (int i = 0; i < 14; ++i) {
        mpuDataBuffer[i] = spi_transfer_byte(0xFF);
    }
    MPU_SPI_CS_OFF();
    #endif // #ifdef MPU_6500

    #ifdef MPU_9250
    MPU_SPI_CS_ON();
    // send register address with Read bit (typically MSB=1 for MPU SPI read)
    (void)spi_transfer_byte((uint8_t)(MPU_ACCEL_XOUT_H | 0x80));
    for (int i = 0; i < 14; ++i) {
        mpuDataBuffer[i] = spi_transfer_byte(0xFF);
    }
    MPU_SPI_CS_OFF();
    #endif // #ifdef MPU_9250

    #ifdef ICM_20948
    ICM_SelectBank(0);
    MPU_SPI_CS_ON();
    // send register address with Read bit (typically MSB=1 for ICM SPI read)
    (void)spi_transfer_byte((uint8_t)(ICM_ACCEL_XOUT_H | 0x80));
    for (int i = 0; i < 21; ++i) {
        if (i < 14) {
            mpuDataBuffer[i] = spi_transfer_byte(0xFF);
        } else {
            // 磁力计数据自动存放在 EXT_SLV_SENS_DATA_00 (0x3B) 开始的寄存器中
            magDataBuffer[i - 14] = spi_transfer_byte(0xFF);
        }
    }
    MPU_SPI_CS_OFF();
    #endif // #ifdef ICM_20948

    #endif // #ifdef COMMUNICATION_TYPE_SPI
}

void Read_BMP_All(void) {
    #ifdef BMP280
    #ifdef COMMUNICATION_TYPE_SPI
    BMP_SPI_CS_ON();
    // send register address with Read bit (typically MSB=1 for BMP SPI read)
    (void)spi_transfer_byte((uint8_t)(0xF7 | 0x80)); // Example register for BMP data
    for (int i = 0; i < 6; ++i) { // assuming 6 bytes of data
        bmpDataBuffer[i] = spi_transfer_byte(0xFF);
    }
    BMP_SPI_CS_OFF();
    uint32_t adc_P = ((uint32_t)bmpDataBuffer[0] << 12) | ((uint32_t)bmpDataBuffer[1] << 4) | ((bmpDataBuffer[2] >> 4) & 0x0F);
    uint32_t adc_T = ((uint32_t)bmpDataBuffer[3] << 12) | ((uint32_t)bmpDataBuffer[4] << 4) | ((bmpDataBuffer[5] >> 4) & 0x0F);
    BMP_Compensate_T(adc_T);
    BMP_Compensate_P(adc_P);
    BMP_GetAltitude();
    #endif // #ifdef COMMUNICATION_TYPE_SPI
    #endif // #ifdef BMP280
}

void Read_BMP_Calibration(void) {
    uint8_t b[24];
    for (int i = 0; i < 24; ++i) b[i] = Read_BMP_Register(0x88 + i);
    bmp_calib.dig_T1 = (uint16_t)(b[1] << 8 | b[0]);
    bmp_calib.dig_T2 = (int16_t)(b[3] << 8 | b[2]);
    bmp_calib.dig_T3 = (int16_t)(b[5] << 8 | b[4]);
    bmp_calib.dig_P1 = (uint16_t)(b[7] << 8 | b[6]);
    bmp_calib.dig_P2 = (int16_t)(b[9] << 8 | b[8]);
    bmp_calib.dig_P3 = (int16_t)(b[11] << 8 | b[10]);
    bmp_calib.dig_P4 = (int16_t)(b[13] << 8 | b[12]);
    bmp_calib.dig_P5 = (int16_t)(b[15] << 8 | b[14]);
    bmp_calib.dig_P6 = (int16_t)(b[17] << 8 | b[16]);
    bmp_calib.dig_P7 = (int16_t)(b[19] << 8 | b[18]);
    bmp_calib.dig_P8 = (int16_t)(b[21] << 8 | b[20]);
    bmp_calib.dig_P9 = (int16_t)(b[23] << 8 | b[22]);
}

void BMP_Compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
}

void BMP_Compensate_P(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)bmp_calib.dig_P1) >> 33;
    if (var1 == 0) return; // avoid div0
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    barometricPressure = p / 25600.0f + BAROMETERIC_PRESSURE_OFFSET; // convert fixed-point to float Pa
}

void BMP_GetAltitude() {
    float T = temperature + 273.15f; 
    // 0.190263 是 R*L/g 的常数
    const float EXP = 0.190263f; 
    
    // 使用实时温度 T 替代固定常数 44330 (44330 实际上包含了 288.15K 的假设)
    // 高度 = (T / 0.0065) * (1 - (P/P0)^EXP)
    altitude = (T / 0.0065f) * (1.0f - powf(barometricPressure / SEA_LEVEL_PRESSURE_HPA, EXP));
}

void ICM_SelectBank(uint8_t bank) {
#ifdef ICM_20948
    // ICM-20948 的 REG_BANK_SEL 地址是 0x7F，Bank 值位于位 [5:4]
    Write_MPU_Register(REG_BANK_SEL, (bank << 4) & 0x30);
#endif
}