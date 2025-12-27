# ifndef __USART1_H
# define __USART1_H

# include "stm32f4xx.h"
# include <string.h>

# define BUADRATE_9600     9600

# define RX_BUFFER_SIZE     64
# define TX_BUFFER_SIZE     64
# define RX_STATE_IDLE      0
# define RX_STATE_COMPLETE  1

extern uint8_t rxBufferUart1[RX_BUFFER_SIZE];
extern uint8_t txBufferUart1[TX_BUFFER_SIZE];
extern __IO uint16_t rxIndexUart1;
extern __IO uint8_t rxStatusUart1;

void Init_USART1(uint32_t baudrate);
uint16_t Read_USART1_Data(uint8_t* data);
void Write_USART1_Data(uint8_t* data, uint16_t size);

# endif /* __USART1_H */