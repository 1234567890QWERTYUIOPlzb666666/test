/** 
  * @file     modeswitch_task.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    底层串口配置，
	*
  *	@author   Fatmouse
  *
  */
	
#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"
typedef enum
{
  MANUAL_CTRL_MODE,
  SEMI_AUTO_MODE,
  AUTO_CTRL_MODE,
  SAFETY_MODE,
} infantry_mode_e;

typedef enum
{
  PROTECT_MODE,       	     
	LOCK_MODE,					//云台锁定在中点，底盘自身运动			
  SEPARATE_MODE,      //底盘云台分离，底盘跟随云台
	ENERGY_MODE,
	DANCE_MODE,         //摇摆模式  
  REMOTER_MODE,  
	KEYBOARD_MODE,      //键盘模式
	VISION_MODE,        //视觉辅助模式
} ctrl_mode_e;

extern ctrl_mode_e ctrl_mode;
extern ctrl_mode_e last_ctrl_mode;
void mode_switch_task(void const *argu);
void get_sw_mode(void);
void unlock_init(void);
	
extern uint8_t flag_unlock;
#endif
