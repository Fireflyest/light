# include "usystem.h"

__IO uint32_t uwTimingDelay;

/**
 * * @brief  Inserts a delay time.
 * * @param  nTime: specifies the delay time length, in ms.
 * * @retval None
 */
void delay_ms(__IO uint32_t nTime) {
    uwTimingDelay = nTime;
    while (uwTimingDelay != 0);
}


void Init_USystem() {
    Init_Key();
    Init_LED();
}

void Init_USART(uint16_t baudrate) {
    Init_DMA_For_USART1_RX(rxBufferUart1, sizeof(rxBufferUart1));
    Init_DMA_For_USART1_TX(txBufferUart1);
    Init_USART1(baudrate);
}

void Init_PWM(uint16_t period, uint16_t prescaler) {
    Init_PWM_TIM(period, prescaler);
    Init_DMA_For_PWM_TIM3(pwmDutyBuffer);
}

void Init_Display() {
    Init_OLED_Hardware();
    Init_GFX();
    UI_Init();
}