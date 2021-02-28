#include "bsp_usart.h"
#include "remote_msg.h"
#include "string.h"
#include "judge.h"
#include "status_task.h"
#include "vision_task.h"

uint8_t dma_dbus_buf[DMA_DBUS_LEN];				
uint8_t	dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_vision_buf[20];
//DBUS����������һ������  it.c����Ҫ��cube�Զ����ɵ��жϺ���ע�͵�������ֻ�ܽ��յ�һ���ֽڡ�ԭ���д��о���
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance== USART1)			//DBUS����  
	{
		rc_callback_handler(&rc,dma_dbus_buf);
		status.dbus_status = 1;
		HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
	}
	
	else if(huart->Instance== UART4)	//JUDGE����
	{
		Judge_Read_Data(dma_judge_buf);
		HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
	}
	
	else if(huart->Instance== USART6)	//VISION����
	{
		vision_read_data(dma_vision_buf);
		HAL_UART_Receive_DMA(huart,dma_vision_buf,20);
	}
}

/**
  * @brief ���ڿ����ж�   ע������it.c��ÿ�����ڵ��ж��е��øú���
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //�ж��Ƿ��ǿ����ж�
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
		HAL_UART_DMAStop(huart);															//ֹͣ����DMA����
		USER_UART_IDLECallback(huart);                        //���ô��ڹ��ܻص�����
	}
}


/**
* @brief  ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void USER_UART_Init()
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);
	
	__HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
	__HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
	
//	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart6,dma_vision_buf,20);
	
}
