#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f4xx.h"
#include "board.h"

#define BMP_CTRL_MEAS      0xF4
#define BMP_CONFIG         0xF5

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

extern bmp_calib_t bmp_calib;
extern uint8_t bmp_rx_buf[6];

void BMP280_Init(void);
void BMP280_Read(void);

#endif /* __BMP280_H */