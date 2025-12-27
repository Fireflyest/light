#ifndef __I2COLED_H
#define __I2COLED_H

#include "stm32f4xx.h"

// OLED Parameters
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// Hardware Initialization
void Init_OLED_Hardware(void);

// Low-level Command Interface
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t data);

// Screen Update Interface
// Sends a specific page (128 bytes) from the buffer to the screen using DMA
void OLED_UpdatePage_DMA(uint8_t pageIdx, uint8_t* pBuffer);

// Check if DMA is currently busy
uint8_t OLED_IsDMABusy(void);

#endif /* __I2COLED_H */