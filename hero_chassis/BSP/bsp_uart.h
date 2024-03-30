#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"

void DMA_Usart_Send(uint8_t *buf,uint8_t len);//串口发送封装
void DMA_Usart1_Read(uint8_t *Data,uint8_t len);//串口接收封装

#endif