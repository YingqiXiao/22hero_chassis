#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"

void DMA_Usart_Send(uint8_t *buf,uint8_t len);//���ڷ��ͷ�װ
void DMA_Usart1_Read(uint8_t *Data,uint8_t len);//���ڽ��շ�װ

#endif