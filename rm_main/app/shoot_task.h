/** 
  * @file     shoot_task.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ������
	*
  *	@author   Fatmouse
  *
  */
#ifdef  __SHOOT_TASK_GLOBALS
#define __SHOOT_TASK_EXT
#else
#define __SHOOT_TASK_EXT extern
#endif

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_FricMotor.h"

#define SHOOT_PERIOD 10
typedef struct
{
	/* position loop ecd*/
	float last_trigger_angle_ref;
  float trigger_angle_ref;
  float trigger_angle_fdb;
	
  /* speed loop */
  float trigger_spd_ref;
  float trigger_spd_fdb;
	
} trigger_pid_t;
typedef struct
{
  uint8_t shoot_mode;
  uint16_t shoot_speed;
  uint8_t shoot_energy;
  uint8_t house_switch;
	trigger_pid_t pid;
	
} shoot_t;

extern shoot_t shoot;

void shoot_task(void const *argu);
void shoot_init(void);
void shoot_mode_sw(void);
#endif
