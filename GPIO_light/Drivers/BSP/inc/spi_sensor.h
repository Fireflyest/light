#ifndef __SPI_SENSOR_H
#define __SPI_SENSOR_H

#include "stm32f4xx.h"
#include "board.h"


typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} SPI_Sensor_HandleTypeDef;

/**
 * @brief Initialize the GPIO pins for SPI2
 * SPI2: SCK (PB13), MISO (PB14), MOSI (PB15)
 * ICM-20948 CS (PB12), BMP280 CS (PA5)
 */
void SPI_Sensor_Init(void);
void SPI_Sensor_Select(SPI_Sensor_HandleTypeDef *handle);
void SPI_Sensor_Deselect(SPI_Sensor_HandleTypeDef *handle);
uint8_t SPI_Sensor_TransferByte(uint8_t tx);

#endif /* __SPI_SENSOR_H */