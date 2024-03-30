#include "usart.h"

/*
*********************************************************************************************************
* �� �� ��: DMA_Usart_Send
* ����˵��: ���ڷ��͹��ܺ���
* ��  ��: buf��len
* �� �� ֵ: ��
*********************************************************************************************************
*/
void DMA_Usart_Send(uint8_t *buf,uint8_t len)//���ڷ��ͷ�װ
{
 if(HAL_UART_Transmit_DMA(&huart1, buf,len)!= HAL_OK) //�ж��Ƿ�����������������쳣������쳣�жϺ���
  {
   Error_Handler();
  }

}

/*
*********************************************************************************************************
* �� �� ��: DMA_Usart1_Read
* ����˵��: ���ڽ��չ��ܺ���
* ��  ��: Data,len
* �� �� ֵ: ��
*********************************************************************************************************
*/
void DMA_Usart1_Read(uint8_t *Data,uint8_t len)//���ڽ��շ�װ
{
	HAL_UART_Receive_DMA(&huart1,Data,len);//���´�DMA����
}

//void USART1_IRQHandler(void)
//{
//	uint32_t tmp_flag = 0;
//	uint32_t temp;
//	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
//	if((tmp_flag != RESET))//idle��־����λ
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
//		//temp = huart1.Instance->SR;  //���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ���
//		//temp = huart1.Instance->DR; //��ȡ���ݼĴ����е�����
//		//������������Ǿ��Ч
//		HAL_UART_DMAStop(&huart1); //
//		temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���   
//		//temp  = hdma_usart1_rx.Instance->NDTR;//��ȡNDTR�Ĵ��� ��ȡDMA��δ��������ݸ�����
//		//���������Ǿ��Ч
//		rx_len =  BUFFER_SIZE - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
//		recv_end_flag = 1;	// ������ɱ�־λ��1	
//	 }
//  HAL_UART_IRQHandler(&huart1);

//}
