#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

#define 	DBUS_HUART    huart1
#define   JUDGE_HUART   huart4		
#define 	GYRO_HUART		huart5		//因最新陀螺仪使用CAN，暂时无用
#define   DEBUG_HUARE		huart6

#define  DMA_DBUS_LEN			18
#define  DMA_JUDGE_LEN		200

union rec_float			//接收float信息
{
	float    msg_float;
	uint8_t  msg_uint8_t[4];
};

union rec_uint16_t
{
	uint16_t msg_uint16_t;
	uint8_t  msg_uint8_t[2];
};

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);

extern uint8_t dma_dbus_buf[DMA_DBUS_LEN];
#endif

