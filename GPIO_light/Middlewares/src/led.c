# include "led.h"

uint8_t __IO ledToggleCmd = LED_TOGGLE_CMD_ALL_OFF;
uint8_t __IO ledToggleCount = 0;

void Init_LED(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the GPIO Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Configure the GPIO pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void LED_On(void) {
    ledToggleCmd = LED_TOGGLE_CMD_ALL_ON;
    ledToggleCount = sizeof(ledToggleCmd) * 8;
}

void LED_Off(void) {
    ledToggleCmd = LED_TOGGLE_CMD_ALL_OFF;
    ledToggleCount = sizeof(ledToggleCmd) * 8;
}

void LED_Blink(uint8_t toggleCmd, uint8_t times) {
    ledToggleCmd = toggleCmd;
    ledToggleCount = sizeof(ledToggleCmd) * 8 * times;
}