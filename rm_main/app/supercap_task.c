/** 
  * @file supercap_task. c
  * @version 1.0
  * @date Mar,24th 2019
	*
  * @brief  ��������
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
	supercap_control.supercap_mode   = 0x01; //�ֶ�ģʽ-�Զ������ݳ��
	supercap_control.supercap_switch = 0x00;//��ع���
	if(rc.kb.bit.SHIFT)
	{
		supercap_control.supercap_switch = 0x01;//�������ݹ���	
	}
	else
	{
		supercap_control.supercap_switch = 0x00;//��ع���
	}
	osSignalSet(can_msg_send_task_t, SUPERCAP_CONTROL_MSG_SEND);
}		

void supercap_init()
{
  supercap_control.supercap_mode   = 0x02; //  0x00:�Զ�ģʽ(����);0x01:�ֶ�ģʽ;0x02:����ģʽ
	supercap_control.supercap_switch = 0x00; //  0x00:�л���ع���;0x01:�л��������ݹ���
}
