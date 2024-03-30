#ifndef __REFEREE_USART_H
#define __REFEREE_USART_H

#include "main.h"
#include "usart.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT     1024
#define BUF_SIZE 1024

#define REFEREE_FIFO_BUF_LENGTH 1024

#define c_huart huart1
#define c_UART USART1
#define c_dma hdma_usart1_rx

extern fifo_s_t referee_fifo;

    extern DMA_HandleTypeDef c_dma;
//    extern uint8_t *rx_p;
    extern uint8_t err;
    extern volatile uint8_t rx_len;
    extern volatile uint8_t recv_end_flag;
    extern uint8_t rx_buff[BUF_SIZE];

void referee_usart_fifo_init(void);
void RE_usart_tx_dma_enable(uint8_t *data, uint16_t len);
void Referee_Init(void);



#endif
