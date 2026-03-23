#include "icm20948.h"

uint8_t imu_tx_buf[IMU_DMA_LEN];
uint8_t imu_rx_buf[IMU_DMA_LEN];
uint8_t mag_rx_buf[6];

static uint8_t SPI_Transfer_Byte(uint8_t tx) {
    // wait TXE
    uint16_t timeout;
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_IMU, SPI_I2S_FLAG_TXE) == RESET; timeout--);
    SPI_I2S_SendData(SPI_IMU, tx);
    // wait RXNE
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_IMU, SPI_I2S_FLAG_RXNE) == RESET; timeout--);
    return (uint8_t)SPI_I2S_ReceiveData(SPI_IMU);
}

static void ICM20948_Register_Write(uint8_t reg, uint8_t data) {
    // cs on
    GPIO_ResetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);
    // send register address (ensure MSB = 0 for write)
    (void)SPI_Transfer_Byte((uint8_t)(reg & 0x7F));
    // send data
    (void)SPI_Transfer_Byte(data);
    // cs off
    GPIO_SetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);
}

static void ICM_SelectBank(uint8_t bank) {
    // ICM-20948 的 REG_BANK_SEL 地址是 0x7F，Bank 值位于位 [5:4]
    ICM20948_Register_Write(REG_BANK_SEL, (bank << 4) & 0x30);
}

static void TIM_Config_For_IMU(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启 TIM2 时钟 (TIM2 在 APB1 总线上)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2. 配置定时器参数 (假设系统主频 84MHz, APB1 频率 42MHz, 但 TIM2/3/4/5 频率会翻倍至 84MHz)
    // 目标频率 = 84,000,000 / (84 * 1000) = 1000 Hz (1ms)
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 3. 配置 TIM2 更新中断 (Update Interrupt)
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 4. 配置 NVIC 中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 优先级低于 SysTick 但高于 DMA 完成中断
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

void ICM20948_Init(void) {
    // SPI2 init (master, Mode 0)
    SPI_I2S_DeInit(SPI2);
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        // Mode 0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // adjust for speed
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);

    ICM_SelectBank(0);
    // 硬件复位
    ICM20948_Register_Write(0x06, 0x80); 
    for (volatile uint32_t i = 0; i < 0x000FFFFF; i++); 
    // 解除睡眠并切换时钟
    ICM20948_Register_Write(0x06, 0x01); 
    
    // 先复位 I2C Master，再启用 (参考 HAL 的 0x22 逻辑)
    // 0x16 = I2C_IF_DIS | I2C_MST_RST | SRAM_RST
    ICM20948_Register_Write(0x03, 0x16); 
    for (volatile uint32_t i = 0; i < 0x0002FFFF; i++); 
    ICM20948_Register_Write(0x03, 0x30); // 正式使能 Master 和禁用从机 I2C

    ICM_SelectBank(3);
    // 开启 P_NSR (Restart) 模式 (Bit 4) + 345.6kHz
    // 0x17 = 0x10 (P_NSR) | 0x07 (Clock)
    ICM20948_Register_Write(0x01, 0x17); 
    ICM20948_Register_Write(0x02, 0x01); // 使能 Slave 0 延迟采样

    // 磁力计软复位
    ICM20948_Register_Write(0x03, 0x0C); // Mag 写地址
    ICM20948_Register_Write(0x04, 0x32); // CNTL3
    ICM20948_Register_Write(0x06, 0x01); // SRST
    ICM20948_Register_Write(0x05, 0x81); // 触发单次写
    for (volatile uint32_t i = 0; i < 0x0006FFFF; i++); 

    // 设置磁力计连续模式
    ICM20948_Register_Write(0x03, 0x0C); 
    ICM20948_Register_Write(0x04, 0x31); // CNTL2
    ICM20948_Register_Write(0x06, 0x08); // Mode 4 (100Hz)
    ICM20948_Register_Write(0x05, 0x81); 
    for (volatile uint32_t i = 0; i < 0x0007FFFF; i++);

    // 从 ST1 (0x10) 开始读，长度设为 9 (含 ST1, 6轴数据, TMPS, ST2)
    ICM20948_Register_Write(0x03, 0x80 | 0x0C); // Mag 读地址
    ICM20948_Register_Write(0x04, 0x10);        // 从 ST1 开始读取
    ICM20948_Register_Write(0x05, 0x89);        // 读取 9 字节

    ICM_SelectBank(2);
    // 配置量程
    ICM20948_Register_Write(0x00, 0x04); // GYRO_SMPLRT_DIV
    ICM20948_Register_Write(0x01, 0x1F); // ±2000dps + DLPF
    ICM20948_Register_Write(0x14, 0x1D); // ±8g + DLPF
    
    ICM_SelectBank(0);

    imu_tx_buf[0] = 0x80 | 0x2D; // 读起始地址 0x2D (Accel_X_H)
    for(int i = 1; i < IMU_DMA_LEN; i++) imu_tx_buf[i] = 0xFF;

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // TIM_Config_For_IMU();
}

void ICM20948_Read(void) {
    // cs on
    GPIO_ResetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

    // send register address with Read bit (typically MSB=1 for ICM SPI read)
    (void)SPI_Transfer_Byte((uint8_t)(ICM_ACCEL_XOUT_H | 0x80));
    for (int i = 0; i < 14; ++i) {
        imu_rx_buf[i] = SPI_Transfer_Byte(0xFF);
    }

    // cs off
    GPIO_SetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

    // cs on
    GPIO_ResetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

    (void)SPI_Transfer_Byte(ICM_EXT_SENS_DATA_00 | 0x80); // 0x49
    uint8_t ext[9];
    for (int i = 0; i < 9; i++) {
        ext[i] = SPI_Transfer_Byte(0xFF);
    }

    // cs off
    GPIO_SetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

    // for (int i = 0; i < 6; i++) {
    //     mag_rx_buf[i] = ext[1 + i];
    // }
    if ((ext[0] & 0x01) && !(ext[8] & 0x08)) {
        for (int i = 0; i < 6; i++) {
            mag_rx_buf[i] = ext[1 + i];
        }
    }
}
