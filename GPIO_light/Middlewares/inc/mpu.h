#ifndef __MPU_H
#define __MPU_H

#include "stm32f4xx.h"

#define	MPU_SMPLRT_DIV		0x19
#define	MPU_CONFIG			0x1A
#define	MPU_GYRO_CONFIG		0x1B
#define	MPU_ACCEL_CONFIG	0x1C

#define	MPU_ACCEL_XOUT_H	0x3B
#define	MPU_ACCEL_XOUT_L	0x3C
#define	MPU_ACCEL_YOUT_H	0x3D
#define	MPU_ACCEL_YOUT_L	0x3E
#define	MPU_ACCEL_ZOUT_H	0x3F
#define	MPU_ACCEL_ZOUT_L	0x40
#define	MPU_TEMP_OUT_H		0x41
#define	MPU_TEMP_OUT_L		0x42
#define	MPU_GYRO_XOUT_H		0x43
#define	MPU_GYRO_XOUT_L		0x44
#define	MPU_GYRO_YOUT_H		0x45
#define	MPU_GYRO_YOUT_L		0x46
#define	MPU_GYRO_ZOUT_H		0x47
#define	MPU_GYRO_ZOUT_L		0x48

#define	MPU_PWR_MGMT_1		0x6B
#define	MPU_PWR_MGMT_2		0x6C
#define	MPU_WHO_AM_I		0x75

#define MPU_ADDRESS         0xD0

// #define COMMUNICATION_TYPE_I2C
#define COMMUNICATION_TYPE_SPI

extern __IO uint8_t mpuReadDone;
extern uint8_t mpuDataBuffer[14];


void Init_MPU_Hardware(void);

void Write_MPU_Register(uint8_t reg, uint8_t data);
void Read_MPU_All();

#endif /* __MPU_H */