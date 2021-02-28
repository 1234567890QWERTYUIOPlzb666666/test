#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"

#define Left_FricMotor_PWM	TIM3->CCR4
#define Right_FricMotor_PWM	TIM12->CCR1
#define Init_PWM	 900
#define init_speed 0      
//#define low_speed 300  
#define low_speed 470 
#define energy_speed 410  
#define high_speed 350
//#define low_speed 555  //11.87~11.97m/s 用于1级步兵下12m/s射速限制。射击距离3.00m左右 557
//#define low_speed 560  //13.73~13.96m/s 用于2级步兵下14m/s射速限制。射击距离3.60m左右
//#define low_speed 595  //15.67~15.97m/s 用于3级步兵下16m/s射速限制。射击距离4.20m左右
//#define low_speed 696  //19.67~19.97m/s 用于3级步兵下20m/s射速限制。射击距离5m左右
typedef enum
{
  FRIC_SPEED_DEFAULT = 0x00,
  FRIC_SPEED_ENERGY  = 0x01,
  FRIC_SPEED_VISION  = 0x02,
	FRIC_SPEED_STOP    = 0x03,
	FRIC_SPEED_HIGH    = 0x04,
	FRIC_SPEED_LOW     = 0x05,
} fric_speed_e;

void FricMotor_init(void);
void fricmotor_status(void);
void FricGunControl(uint16_t pwm1,uint16_t pwm2);
void laser_on(void);
void laser_off(void);
void TriggerMotor_runback(void);
#endif
