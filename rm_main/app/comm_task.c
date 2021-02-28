/** 
  * @file     comm_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    交互任务文件
	*
  *	@author   Fatmouse
  *
  */
#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "pid.h"
#include "modeswitch_task.h"

motor_current_t motor_cur;


/**
  * @brief can_msg_send_task
  * @param 
  * @attention  
  * @note  
  */
	
void can_msg_send_task(void const *argu)
{
	osEvent event;
	for(;;)
  {
		event = osSignalWait(GIMBAL_MOTOR_MSG_SEND  | \
		                     CHASSIS_MOTOR_MSG_SEND | \
		                     SHOOT_CONTROL_MSG_SEND | \
		                     SUPERCAP_CONTROL_MSG_SEND, osWaitForever);
		if (event.status == osEventSignal)
    {
			if(ctrl_mode ==PROTECT_MODE)
			{
				for(int i=0;i<4;i++)		motor_cur.chassis_cur[i]=0;
				for(int i=0;i<3;i++)		motor_cur.gimbal_cur[i]=0;
			}
			if(flag_unlock)
			{
				if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
				{
					can1_send_message(GIMBAL_CAN_TX_ID, -motor_cur.gimbal_cur[0],-motor_cur.gimbal_cur[1],motor_cur.gimbal_cur[2],0);
					can2_send_message(GIMBAL_CAN_TX_ID, -motor_cur.gimbal_cur[0],-motor_cur.gimbal_cur[1],motor_cur.gimbal_cur[2],0);
				}
				if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
				{
					can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur[0],motor_cur.chassis_cur[1],motor_cur.chassis_cur[2],motor_cur.chassis_cur[3]);
				}
			}
		}
	}
}


