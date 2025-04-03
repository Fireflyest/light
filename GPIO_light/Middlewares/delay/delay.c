#include "delay.h"

static __IO uint32_t uwTimingDelay;

/*********************************************************************
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
*********************************************************************/
void Delay_Decrement(void) {
    if (uwTimingDelay != 0x00) {
        uwTimingDelay--;
    }
}

/********************************************************************
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in ms.
  * @retval None
  * @note   The delay is based on SysTick timer.
  *         This function is blocking.
********************************************************************/
void delay_ms(__IO uint32_t nTime) {
    uwTimingDelay = nTime;

    while (uwTimingDelay != 0);
}

void delay_us(__IO uint32_t nTime) {
    // Convert microseconds to milliseconds if it's large enough
    if (nTime >= 1000) {
      delay_ms(nTime / 1000);
      nTime %= 1000;
    }
    // For small microsecond delays
    if (nTime > 0) {
      uint32_t ticks = nTime * (SystemCoreClock / 1000000);
      uint32_t start = DWT->CYCCNT;
      while ((DWT->CYCCNT - start) < ticks);
    }
}