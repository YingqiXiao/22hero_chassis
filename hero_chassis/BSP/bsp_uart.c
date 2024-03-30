#include "usart.h"

/*
*********************************************************************************************************
* 函 数 名: DMA_Usart_Send
* 功能说明: 串口发送功能函数
* 形  参: buf，len
* 返 回 值: 无
*********************************************************************************************************
*/
void DMA_Usart_Send(uint8_t *buf,uint8_t len)//串口发送封装
{
 if(HAL_UART_Transmit_DMA(&huart1, buf,len)!= HAL_OK) //判断是否发送正常，如果出现异常则进入异常中断函数
  {
   Error_Handler();
  }

}

/*
*********************************************************************************************************
* 函 数 名: DMA_Usart1_Read
* 功能说明: 串口接收功能函数
* 形  参: Data,len
* 返 回 值: 无
*********************************************************************************************************
*/
void DMA_Usart1_Read(uint8_t *Data,uint8_t len)//串口接收封装
{
	HAL_UART_Receive_DMA(&huart1,Data,len);//重新打开DMA接收
}

//void USART1_IRQHandler(void)
//{
//	uint32_t tmp_flag = 0;
//	uint32_t temp;
//	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //获取IDLE标志位
//	if((tmp_flag != RESET))//idle标志被置位
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
//		//temp = huart1.Instance->SR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
//		//temp = huart1.Instance->DR; //读取数据寄存器中的数据
//		//这两句和上面那句等效
//		HAL_UART_DMAStop(&huart1); //
//		temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// 获取DMA中未传输的数据个数   
//		//temp  = hdma_usart1_rx.Instance->NDTR;//读取NDTR寄存器 获取DMA中未传输的数据个数，
//		//这句和上面那句等效
//		rx_len =  BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//		recv_end_flag = 1;	// 接受完成标志位置1	
//	 }
//  HAL_UART_IRQHandler(&huart1);

//}
