#include "bmp280.h"


bmp_calib_t bmp_calib;
uint8_t bmp_rx_buf[6];

static uint8_t SPI_Transfer_Byte(uint8_t tx) {
    // wait TXE
    uint16_t timeout;
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_IMU, SPI_I2S_FLAG_TXE) == RESET; timeout--);
    SPI_I2S_SendData(SPI_IMU, tx);
    // wait RXNE
    for (timeout = 0xFFFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_IMU, SPI_I2S_FLAG_RXNE) == RESET; timeout--);
    return (uint8_t)SPI_I2S_ReceiveData(SPI_IMU);
}

static void BMP280_Register_Write(uint8_t reg, uint8_t data) {
    // cs on
    GPIO_ResetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    // send register address (ensure MSB = 0 for write)
    (void)SPI_Transfer_Byte((uint8_t)(reg & 0x7F));
    // send data
    (void)SPI_Transfer_Byte(data);
    // cs off
    GPIO_SetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
}

static void BMP280_Calibration_Read(void) {
    uint8_t b[24];
    
    // cs on
    GPIO_ResetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    
    // send start register address with Read bit (0x88 | 0x80 = 0x88 starting address)
    (void)SPI_Transfer_Byte(0x88 | 0x80);
    
    // continuous read 24 bytes
    for (int i = 0; i < 24; ++i) {
        b[i] = SPI_Transfer_Byte(0xFF);
    }
    
    // cs off
    GPIO_SetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);

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

uint8_t BMP280_Register_Read(uint8_t reg) {
    uint8_t v;
    // cs on
    GPIO_ResetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    for (volatile int i = 0; i < 200; ++i); // short settle delay
    (void)SPI_Transfer_Byte((uint8_t)(reg | 0x80)); // read flag
    v = SPI_Transfer_Byte(0xFF);
    // cs off
    GPIO_SetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    return v;
}

void BMP280_Read(void) {
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

    uint16_t timeout;
    for (timeout = 0xFFFF; timeout > 0 && spi2Occupied != 0; timeout--);
    for (timeout = 0xFFFF; timeout > 0 && GPIO_ReadOutputDataBit(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN) == 0; timeout--);

    spi2Occupied = 2; // mark SPI2 as occupied by BMP280
    // cs on
    GPIO_ResetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    // send register address with Read bit (typically MSB=1 for BMP SPI read)
    (void)SPI_Transfer_Byte((uint8_t)(0xF7 | 0x80)); // Example register for BMP data
    for (int i = 0; i < 6; ++i) { // assuming 6 bytes of data
        bmp_rx_buf[i] = SPI_Transfer_Byte(0xFF);
    }
    // cs off
    GPIO_SetBits(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);
    spi2Occupied = 0; // release SPI2

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void BMP280_Init(void) {
    BMP280_Register_Write(BMP_CTRL_MEAS, 0x27); // normal mode, temp and pressure oversampling x1
    BMP280_Register_Write(BMP_CONFIG, 0xA0);    // standby 1000ms, filter off
    BMP280_Calibration_Read();
}