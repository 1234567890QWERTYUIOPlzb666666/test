/** 
  * @file     bsp_TriggerMotor.c
  * @version  v2.0
  * @date     July,8th 2019
	*
  * @brief    拨弹电机基础配置包含1.拨弹电机PID初始化
	*								          2.拨弹电机控制
	*								          3.拨弹电机速度位置闭环控制
	*
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
	
#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "math_calcu.h"

uint8_t single_flag;
heat_limit_t heat_limit;
int32_t t,p;
uint8_t ML_key_up = 1;
uint8_t back_flag= 0;			//回转状态位
/**
  * @brief          拨弹电机初始化，初始化PID，电机指针
  * @author         Fatmouse
  * @param[in]      void
  * @retval         返回空
  */

void TriggerMotor_init(void)
{
	PID_struct_init(&pid_trigger_spd, POSITION_PID, 28000, 15000,
									4.0f,0.005f,0.0f); 
	PID_struct_init(&pid_trigger_angle, POSITION_PID, 4320, 200,
									0.3f,0.001f,0.0f); 
}


void TriggerMotor_status(void)
{
	if(rc.sw2 == RC_UP)
	{
		single_flag =1;
	}
	if(rc.sw2 == RC_MI&&single_flag)
	{
		shoot.shoot_mode = CONTROL_MODE_BULLET_SINGLE;
		single_flag =0;
	}
	if(rc.sw2 == RC_DN)
	{
		shoot.shoot_mode = CONTROL_MODE_BULLET;
	}
}
/**
  * @brief          拨弹电机PID控制
  * @param[in]      void
  * @retval         void
  */

void TriggerMotor_control(void)
{
//	Judgement_Heat_data_Update();
//	switch (shoot.shoot_mode)
//	{
//		case CONTROL_MODE_BULLET:
//		{
//			t = motor_trigger.total_angle;
//			p = shoot.pid.last_trigger_angle_ref;
//			if(ABS(shoot.pid.trigger_angle_ref-shoot.pid.trigger_angle_fdb) < 1000 && !back_flag )
//			{
//				if(TIM3->CCR4 >= 1000)
//				{
//					shoot.pid.trigger_angle_ref += 36859;
//					shoot.pid.last_trigger_angle_ref = shoot.pid.trigger_angle_ref;
//				}
//			}				
//		}break;
//		case CONTROL_MODE_BULLET_SINGLE:
//		{
//			if(TIM3->CCR4 >= 1000)
//			{
//				if(rc.mouse.l && ML_key_up) 
//				{
//					ML_key_up = 0;
//					shoot.pid.trigger_angle_ref += 36859;
//					shoot.pid.last_trigger_angle_ref  = shoot.pid.trigger_angle_ref;
//				}
//			}
//		}break;
//		case CONTROL_MODE_STOP:
//		{		
//		}break;
//	  default:
//		{
//		}break;
//	}
////	TriggerMotor_runback();
//	shoot.pid.trigger_angle_fdb = motor_trigger.total_angle;
//	pid_calc(&pid_trigger_angle, shoot.pid.trigger_angle_fdb,shoot.pid.trigger_angle_ref);//编码器环
//	
//	shoot.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
//	shoot.pid.trigger_spd_ref = pid_trigger_angle.pos_out;
//	pid_calc(&pid_trigger_spd, shoot.pid.trigger_spd_fdb,shoot.pid.trigger_spd_ref);			//速度环
//	
//	gimbal.current[2] = pid_trigger_spd.pos_out;
	switch (shoot.shoot_mode)
	{
		case CONTROL_MODE_BULLET:
		{
			if(TIM3->CCR4 >= 1000)
			{
				shoot.pid.trigger_spd_ref = -3000;
				shoot.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
				pid_calc(&pid_trigger_spd, shoot.pid.trigger_spd_fdb,shoot.pid.trigger_spd_ref);			//速度环
				gimbal.current[2] = pid_trigger_spd.pos_out;
			}
			else	gimbal.current[2] = 0;
			break;
		}
		
		case CONTROL_MODE_BULLET_SINGLE:
		{
			if(TIM3->CCR4 >= 1000)
			{
				if(rc.mouse.l && ML_key_up) 
				{
					ML_key_up = 0;
					shoot.pid.trigger_angle_ref += 36859;
					shoot.pid.last_trigger_angle_ref  = shoot.pid.trigger_angle_ref;
				}
			}
			break;
		}
		case CONTROL_MODE_STOP:
		{
			shoot.pid.trigger_angle_ref = motor_trigger.speed_rpm;		
			shoot.pid.trigger_angle_fdb = motor_trigger.speed_rpm;
			gimbal.current[2] = 0;
			break;
		}
	}
	
}

///**
//  * @brief          热量数据更新
//  * @author         link
//  * @param[in]      void
//  * @retval         void
//  */
//void Judgement_Heat_data_Update(void)
//{   
//		//机器人id
//		heat_limit.robot_id=judge_recv_mesg.robot_state_data.robot_id;
//		
//		//当前热量
//		heat_limit.shoot_current_heat=judge_recv_mesg.power_heat_data.shooter_heat0;
//		
//		//冷却速率
//		heat_limit.shoot_heat_cooling_rate=judge_recv_mesg.robot_state_data.shooter_heat0_cooling_rate;
//		
//		//热量临界
//		heat_limit.shoot_heat_cooling_limit=judge_recv_mesg.robot_state_data.shooter_heat0_cooling_limit;
//}

void TriggerMotor_runback(void)
{
//	uint8_t flag;
	static uint16_t total_ecd;
	static uint16_t ecd_speed;
	static uint16_t run_cnt;
	static uint16_t back_cnt;
	if(rc.sw2 == RC_DN)
	{
		total_ecd += circle_error(motor_trigger.ecd,motor_trigger.last_ecd,8191);
		run_cnt++;
		if(back_flag)  back_cnt++;
		if(run_cnt >=200)
		{
			ecd_speed = total_ecd;
			total_ecd = 0;
			run_cnt=0;
			if(ecd_speed<=100 && !back_flag) 
			{
				shoot.pid.trigger_angle_ref -= 73718;
				back_flag =1;
				back_cnt  =0;
			}
		}
		if(back_cnt>=1500)
		{
			back_cnt =0;
			back_flag =0;
		}
  }
}
