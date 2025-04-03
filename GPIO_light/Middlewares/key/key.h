#ifndef __KEY_H
#define __KEY_H	 

#include "stm32f4xx_conf.h"

#define  K0_ID 	  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)	//PA0--对应K0按键

#define K0_PRES   1

void KEY_Init(void);	//IO初始化

u8 KEY_Scan(u8);  		//按键扫描函数	

#endif

