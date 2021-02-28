/** 
  * @file     bsp_FricMotor.c
  * @version  v2.1
  * @date     July,8th 2019
	*
  * @brief    ����1.Ħ���ֵ���ĳ�ʼ��
									2.Ħ���ֵ��PWM����
									3.Ħ���ֵ��б������
                  4.Ħ����ת�ٷ���
									5.Ħ����PID����
	*
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
Slope_Struct shoot_Fric_pwm_L;
Slope_Struct shoot_Fric_pwm_R; 
uint16_t last_pwm_r;
uint16_t last_pwm_l;
void FricMotor_init(void)
{ 
	//950-2000
	//����ʱ�����Ŵ����
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1); 	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2); 
	
  Left_FricMotor_PWM  = Init_PWM;
	Right_FricMotor_PWM = Init_PWM;
	shoot_Fric_pwm_L.limit_target = Init_PWM;//��ʼ��Ħ����б�º���
	shoot_Fric_pwm_L.real_target  = Init_PWM  ;
	shoot_Fric_pwm_L.change_scale = 0.3;
	shoot_Fric_pwm_R.limit_target = Init_PWM;//��ʼ��Ħ����б�º���
	shoot_Fric_pwm_R.real_target  = Init_PWM  ;
	shoot_Fric_pwm_R.change_scale = 0.3;
	
}


/**
	*@func   		void FricGunControl(uint8_t Control)
	*@bref			Ħ������ͣ
	*@param[in] Control��0Ϊֹͣ��1Ϊ����
  *@retval    void
	*@note			900������ת 
	*/
void FricGunControl(uint16_t pwm1,uint16_t pwm2)
{
	shoot_Fric_pwm_L.limit_target=Init_PWM+pwm1;
	shoot_Fric_pwm_R.limit_target=Init_PWM+pwm2;
	
	Slope_On(&shoot_Fric_pwm_L); //��Ħ����б������
	Slope_On(&shoot_Fric_pwm_R); //��Ħ����б������ 
	
	Right_FricMotor_PWM = shoot_Fric_pwm_R.real_target;
	Left_FricMotor_PWM  = shoot_Fric_pwm_L.real_target;
}     

void fricmotor_status(void)
{
	switch (shoot.shoot_speed)
	{
		case FRIC_SPEED_ENERGY:
		{

			laser_on();
			FricGunControl(energy_speed,energy_speed);

		}break;
		case FRIC_SPEED_HIGH:
		{
			laser_on();
			FricGunControl(high_speed,high_speed);
		}break;
		case FRIC_SPEED_LOW:
		{
			laser_on();
			FricGunControl(low_speed,low_speed);
		}break;
		case FRIC_SPEED_STOP:
		{
			laser_off();
			FricGunControl(init_speed,init_speed);
		}break;
		default:
		{
		}
	}
}



/****************** laser references ********************/
void laser_on(void)
{
	//HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}
void laser_off(void)
{
//  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
