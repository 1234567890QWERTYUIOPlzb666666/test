/**
  * @file     bsp_TriggerMotor.h
  * @version  v2.1
  * @date     July,8th 2019
  *
  * @brief    
  *
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */	
#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#include "pid.h"
#include "main.h"
#include "bsp_can.h"

typedef struct
{
	uint8_t robot_id;
	  
	uint16_t shoot_current_heat; 

	uint16_t shoot_heat_cooling_rate;
	uint16_t shoot_heat_cooling_limit;

	uint8_t  heat_limit_flag;
	float    bullet_speed;           
} heat_limit_t;

typedef enum
{
	CONTROL_MODE_STOP,    
	CONTROL_MODE_BULLET,
  CONTROL_MODE_BULLET_SINGLE,	
} control_mode_e; 

extern heat_limit_t heat_limit;

void TriggerMotor_init(void);
void TriggerMotor_status(void);
void TriggerMotor_control(void);
void Judgement_Heat_data_Update(void);
#endif




