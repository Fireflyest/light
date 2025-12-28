# ifndef __MPU_H
# define __MPU_H

# include "stm32f4xx.h"

# define MPU6050_ADDRESS          0x68
# define MPU6050_RA_ACCEL_XOUT_H  0x3B

extern __IO uint8_t mpuReadDone;
extern uint8_t mpuDataBuffer[14];


void Init_MPU_Hardware(void);

void Read_MPU_All();

# endif /* __MPU_H */