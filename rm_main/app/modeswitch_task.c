#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "supercap_task.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "visionfire_task.h"
#include "remote_msg.h"
#include "cmsis_os.h"
#include "vision_task.h"

#define LASER_UP		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
#define LASER_DOWN	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);	//红外激光

ctrl_mode_e ctrl_mode;
uint8_t flag_unlock=0;

void mode_switch_task(void const *argu)
{
	for(;;)
	{
		if(!flag_unlock)			unlock_init();			//解锁操作
		
		get_sw_mode();
		osDelay(5);
	}
}

static void sw1_mode_handler(void)
{
	switch (rc.sw1)
  {
		case RC_UP:
		{
			ctrl_mode = REMOTER_MODE;
			break;
		}
		case RC_MI:
		{
			ctrl_mode = PROTECT_MODE;
			break;
		}
		case RC_DN:
		{
			ctrl_mode = KEYBOARD_MODE;
			if(rc.mouse.r == 1)		  ctrl_mode = VISION_MODE;
			break;
		}
		default:
		break;
  }

}

static void sw2_mode_handler(void)
{ 
	switch (rc.sw2)
	{
		case RC_UP:
		{
			LASER_DOWN;
			shoot.shoot_speed = FRIC_SPEED_STOP;
			shoot.shoot_mode = CONTROL_MODE_STOP;
		}
		break;
		case RC_MI:
		{
			LASER_UP
		  shoot.shoot_mode = CONTROL_MODE_STOP;
		  shoot.shoot_speed = FRIC_SPEED_LOW;
			if(ctrl_mode == KEYBOARD_MODE)				ctrl_mode = VISION_MODE;
		}break;
		case RC_DN:
		{
			shoot.shoot_mode = CONTROL_MODE_BULLET;
		}break;
		default:
		{
		}
	}
}

void get_sw_mode(void)
{
	sw1_mode_handler();
	sw2_mode_handler();
}

void unlock_init(void)			//解锁函数
{
	if(ctrl_mode == PROTECT_MODE&&shoot.shoot_mode == CONTROL_MODE_STOP)
	{
	 if(rc.ch4==-660)
	 {
		if(rc.ch3==660)
		{
		 flag_unlock = 1;
		}
	 }					
	}
}		
