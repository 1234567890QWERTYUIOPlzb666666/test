/**
  * @file     chassis_task.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief  
  *
  *	@author   Fatmouse
  *
  */
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "stm32f4xx_hal.h"

#define CHASSIS_PERIOD 2
#define CHASSIS_WZ_SET_SCALE	0.0f //底盘速度自旋补偿值
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f		//m3508转化成底盘速度(m/s)的比例
/* chassis parameter structure */
typedef struct 
{
	float vx;
	float vy;
	float vw;
}spd_t;

typedef struct
{
	spd_t     spd_ref;
	spd_t     spd_input;
	spd_t     spd_fdb;
	spd_t     spd_real;    //真实速度
	int16_t   wheel_spd_ref[4];
	int16_t   wheel_spd_input[4];
  int16_t   wheel_spd_fdb[4];
  int16_t   current[4];

  int16_t   position_ref;
	float 	  position_error;
	float     angle_error;
	
	float     wheel_max; 
  float     keyboard_input;	
} chassis_t;

extern chassis_t chassis;
void chassis_task(void const *argu);
void chassis_init(void);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);
void sparate_move(void);
void dance_move(void);
void chassis_speedw_calcu(void);
void chassis_ramp(void);
void chassis_spd_distribution(void);
void chassis_pidout_limit(void);
void chassis_sigmoid(void);

void chassis_odom_send_task(void const * argument);
#endif
