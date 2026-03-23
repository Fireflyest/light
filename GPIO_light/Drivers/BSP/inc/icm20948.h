#ifndef __ICM20948_H
#define __ICM20948_H

#include "stm32f4xx.h"
#include "board.h"

#define ICM_WHO_AM_I        0xEA

#define REG_BANK_SEL        0x7F

#define ICM_GYRO_CONFIG_1   0x01

#define ICM_MOD_CTRL_USR    0x03

#define ICM_PWR_MGMT_1      0x06
#define ICM_PWR_MGMT_2      0x07

#define	ICM_ACCEL_XOUT_H	0x2D
#define	ICM_ACCEL_XOUT_L	0x2E
#define	ICM_ACCEL_YOUT_H	0x2F
#define	ICM_ACCEL_YOUT_L	0x30
#define	ICM_ACCEL_ZOUT_H	0x31
#define	ICM_ACCEL_ZOUT_L	0x32

#define	ICM_GYRO_XOUT_H		0x33
#define	ICM_GYRO_XOUT_L		0x34
#define	ICM_GYRO_YOUT_H		0x35
#define	ICM_GYRO_YOUT_L		0x36
#define	ICM_GYRO_ZOUT_H		0x37
#define	ICM_GYRO_ZOUT_L		0x38

#define	ICM_TEMP_OUT_H		0x39
#define	ICM_TEMP_OUT_L		0x3A

#define ICM_GYRO_CONFIG_1   0x01
#define ICM_ACCEL_CONFIG    0x14

#define ICM_EXT_SENS_DATA_00    0x49

#define	MAG_ADDRESS_ICM		0x0C


#define IMU_DMA_LEN 14
extern uint8_t imu_tx_buf[IMU_DMA_LEN];
extern uint8_t imu_rx_buf[IMU_DMA_LEN];
extern uint8_t mag_rx_buf[6];

void ICM20948_Init(void);
void ICM20948_Read(void);

# endif /* __ICM20948_H */