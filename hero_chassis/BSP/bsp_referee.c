#include "bsp_referee.h"

#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "referee.h"
#include "usart.h"
#include "referee_UI.h"

//接收原始数据，防止DMA传输越界
uint8_t usart1_buf[USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
// Error flag
// 错误标志位
uint8_t err;

// 接收一帧数据的长度
volatile uint8_t rx_len = 0;
// 一帧数据接收完成标志
volatile uint8_t recv_end_flag = 0;

// Define the serial port receiving buffer
//  定义串口接收缓存区
uint8_t rx_buff[BUF_SIZE] = {0};

void Referee_Init(void)
{
	// 不加收不到数据
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, rx_buff, BUF_SIZE);
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{

	uint32_t tmp_flag = 0;
	uint32_t temp;

	// Get the IDLE flag bit
	tmp_flag = __HAL_UART_GET_FLAG(&c_huart, UART_FLAG_IDLE);

	if (tmp_flag != RESET)
	{
		// Clear the IDLE flag in UART
		__HAL_UART_CLEAR_IDLEFLAG(&c_huart);

		// Clear the status register (SR)
		temp = c_huart.Instance->SR;

		// Read data from DR (Data Register)
		temp = c_huart.Instance->DR;

		HAL_UART_DMAStop(&c_huart); // Stop DMA transfer

		// Get the number of untransmitted data in DMA
		temp = c_dma.Instance->CNDTR;

		// Calculate the number of received data by subtracting the total count from the untransmitted data count
		rx_len = BUF_SIZE - temp;

		// Set the receive completion flag to 1
		recv_end_flag = 1;
		
		fifo_s_puts(&referee_fifo, (char*)rx_buff, rx_len);		
		
		referee_unpack_fifo_data();
		
		recv_end_flag = 0;
		
		HAL_UART_Receive_DMA(&huart1, rx_buff, BUF_SIZE);
		
	}
	HAL_UART_IRQHandler(&c_huart);

		

}
