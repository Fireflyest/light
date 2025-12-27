# include "usart1.h"

uint8_t rxBufferUart1[RX_BUFFER_SIZE];
uint8_t txBufferUart1[TX_BUFFER_SIZE];
__IO uint16_t rxIndexUart1 = 0;
__IO uint8_t rxStatusUart1 = RX_STATE_IDLE;


void Init_USART1(uint32_t baudrate) {

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // For UART pins on GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable USART1 clock

    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure USART1 pins: TX (PB6) and RX (PB7) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // Alternate Function mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // Push-pull for UART
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // Pull-up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Connect PB6 and PB7 to USART1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART1, DISABLE);
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    // Enable USART1 RX interrupt
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

uint16_t Read_USART1_Data(uint8_t* data) {
    uint16_t size = rxIndexUart1;
    if (size > 0) {
        memcpy(data, rxBufferUart1, size);
        data[size] = '\0'; // Null-terminate the string
    }
    rxStatusUart1 = RX_STATE_IDLE;
    rxIndexUart1 = 0;
    return size;
}

void Write_USART1_Data(uint8_t* data, uint16_t size) {
    if (size == 0) return;

    uint16_t timeout;
    for (timeout = 0xFFFF;DMA_GetCmdStatus(DMA2_Stream7) != DISABLE && timeout > 0; timeout--) ;
    if (timeout == 0) return;

    uint16_t sendLen = (size > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : size;

    memcpy(txBufferUart1, data, sendLen);

    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
    DMA_SetCurrDataCounter(DMA2_Stream7, sendLen);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}