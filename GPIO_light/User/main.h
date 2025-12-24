/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/main.h
 * @author  MCD Application Team
 * @version V1.8.0
 * @date    04-November-2016
 * @brief   Header for main.c module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __main_H
#define __main_H

/* Includes ------------------------------------------------------------------*/
#include "RTE_Components.h"
#include "stm32f4xx.h"

#include "delay.h"
#include "gpio.h"
#include "key.h"
#include "ui.h"

extern u8 key;           // 保存键值
extern u8 rxBuffer[32];  // 接收缓冲区
extern u8 rxIndex;       // 接收索引
extern u8 txData[16];    // 发送测试数据
extern u8 uartStatus;    // UART状态：0-空闲，1-发送完成，2-接收到数据

#define LED_PORT GPIOC
#define LED_PIN GPIO_Pin_13

// #define PWM_PERIOD     1000    // PWM 周期
// #define PWM_PRESCALER  84      // 84MHz 时钟下，分频为 84 得到 1MHz 的计数频率

#define PWM_PERIOD     20000    // PWM 周期
#define PWM_PRESCALER  84      // 84MHz 时钟下，分频为 84 得到 1MHz 的计数频率

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
