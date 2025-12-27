

#include "key.h"
#include "usystem.h"


__IO uint8_t keyStatus = KEY_STATE_RELEASED;
__IO uint8_t keyEnable = KEY_DISABLE;

void Init_Key(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA,时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           // WK_UP对应引脚PA0--K0按键
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        // 普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);              // 初始化GPIOA

    keyEnable = KEY_ENABLE;
}

uint8_t Key_Status() {
    return keyStatus;
}
