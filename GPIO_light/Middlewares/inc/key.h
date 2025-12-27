#ifndef __KEY_H
#define __KEY_H	 

#include "stm32f4xx_conf.h"

# define KEY_DISABLE         0
# define KEY_ENABLE          1

# define KEY_STATE_RELEASED    0
# define KEY_STATE_PRESSED     1

# define KEY_DEBOUNCE_TIME    20

extern __IO uint8_t keyEnable;
extern __IO uint8_t keyStatus;

void Init_Key(void);
uint8_t Key_Status();

#endif

