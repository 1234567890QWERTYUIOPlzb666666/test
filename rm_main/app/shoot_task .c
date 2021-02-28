/** 
  * @file     shoot_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    
	*
  *	@author   Fatmouse
  *
  */
#define __SHOOT_TASK_GLOBALS
#include "shoot_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "usart.h"

shoot_t shoot;
extern TaskHandle_t can_msg_send_task_t;
extern uint8_t ML_key_up;
void shoot_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		shoot_mode_sw();							//Éä»÷Ä£Ê½ÇÐ»»
		CoverServo_switch();					//¿ª¹Øµ¯²Ö¸Ç
		fricmotor_status();						//Ä¦²ÁÂÖ
		if(ctrl_mode != PROTECT_MODE)	
		{
			TriggerMotor_control();			//²¦µ¯
		}

		osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
	}
}

void shoot_init()
{
  shoot.shoot_mode = CONTROL_MODE_STOP;
	CoverServo_init();
	FricMotor_init();
	TriggerMotor_init();
}

void shoot_mode_sw()
{
	if(ctrl_mode==KEYBOARD_MODE)
	{
		if(rc.mouse.l)
		{
			shoot.shoot_mode = CONTROL_MODE_BULLET;
		}
		else
		{
			shoot.shoot_mode = CONTROL_MODE_STOP;			
		}	
		if(!rc.mouse.l)
		{
			ML_key_up = 1;
		}
	}
}
