/** 
  * @file supercap_task. c
  * @version 1.0
  * @date Mar,24th 2019
	*
  * @brief  超级电容
	*
  *	@author  linking
  *
  */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "remote_msg.h"
#include "supercap_task.h"
#include "bsp_can.h"
#include "math.h"

extern TaskHandle_t can_msg_send_task_t;

void supercap_task(void const *argu)
{
	supercap_control.supercap_mode   = 0x01; //手动模式-自动给电容充电
	supercap_control.supercap_switch = 0x00;//电池供电
	if(rc.kb.bit.SHIFT)
	{
		supercap_control.supercap_switch = 0x01;//超级电容供电	
	}
	else
	{
		supercap_control.supercap_switch = 0x00;//电池供电
	}
	osSignalSet(can_msg_send_task_t, SUPERCAP_CONTROL_MSG_SEND);
}		

void supercap_init()
{
  supercap_control.supercap_mode   = 0x02; //  0x00:自动模式(保留);0x01:手动模式;0x02:保护模式
	supercap_control.supercap_switch = 0x00; //  0x00:切换电池供电;0x01:切换超级电容供电
}
