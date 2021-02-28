/**
	* @file bsp_T_imu.c
	* @version 1.0
	* @date 2020.1.4
  *
  * @brief  TaurusÍÓÂÝÒÇÄÚ²â°æ
  *
  *	@author YY
  *
  */
	
#include "bsp_T_imu.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "KalmanFilter.h"
#include "usart.h"

float 	palstance_buffer[2];
float 	angle_buffer[2];
uint8_t vision_tx_data[6];	//
Taurus_imu_data_t   imu_data;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data)
{
	switch(can_id)
  {
		case TIMU_PALSTANCE_ID:	//½ÇËÙ¶È
		{	
			memcpy(palstance_buffer,CAN_Rx_data,8);
			imu_data.wy = palstance_buffer[0];
			imu_data.wz = palstance_buffer[1];
			memcpy((vision_tx_data+1),(CAN_Rx_data+4),4);
			vision_tx_data[0]=0x11;
			vision_tx_data[5]=0x22;
//			HAL_UART_Transmit_DMA(&huart6,vision_tx_data,9);
		}
		break;
		case TIMU_ANGLE_ID:			//½Ç¶È
		{
			memcpy(angle_buffer,CAN_Rx_data,8);
			imu_data.pitch = angle_buffer[0];
			imu_data.yaw 	 = angle_buffer[1];
		}
		break;
	}
}
