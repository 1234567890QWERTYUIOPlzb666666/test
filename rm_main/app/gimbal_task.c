#define __GIMBAL_TASK_GLOBALS
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "vision_task.h"
#include "KalmanFilter.h"

extern TaskHandle_t can_msg_send_task_t;

gimbal_t gimbal;

void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));

  PID_struct_init(&pid_pit_ecd, POSITION_PID, 5000, 0,
                  10.0f, 0.0f, 0.0f); 		
  PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,25000,
                  15.0f, 0.5f, 0.0f);	
	
	PID_struct_init(&pid_yaw_angle, POSITION_PID, 5000, 0,
                  300.0f, 0.0f, 0.0f);		
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 25000,
									20.0f,0.5f, 0.0f);			
									
	PID_struct_init(&pid_vision_yaw,POSITION_PID,180,100,3.0f,0.0f,0.0f);
	PID_struct_init(&pid_vision_pit,POSITION_PID,500,100,100.0f,0.0f,0.0f);	

	gimbal.pit_center_offset = gimbal_pit_center_offset;
	gimbal.yaw_center_offset = gimbal_yaw_center_offset;

}


float vision_yaw_perdict;			//yaw轴预测量
float vision_yaw_kal_perdict;
/**
  * @brief gimbal_task
  * @param     
  * @attention  
	* @note  
  */
void gimbal_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		switch(ctrl_mode)
		{
			case PROTECT_MODE:
			{
				gimbal.pid.pit_ecd_ref = gimbal_pit_center_offset;
				gimbal.pid.yaw_angle_ref = imu_data.yaw;
				 // gimbal.pid.yaw_angle_ref=moto_yaw.ecd*360/8191;
				for(int i=0;i<3;i++)	gimbal.current[i]=0;
				break;
			}	
			
			case REMOTER_MODE:
			{
				gimbal.pid.pit_relative_ecd   = -1.0f * rc.ch4 * rc_ch4_scale;		
				gimbal.pid.yaw_relative_angle = rc.ch3 * rc_ch3_scale;	
				normal_calcu();
				break;
			}	
			
			case VISION_MODE:
			{
				if(vision.distance)		//视觉模式下捕获到目标
				{
					vision.lost_cnt = 0;		
					vision_calcu();													
				}
				else if(vision.distance == 0 && vision.lost_cnt < 5)		//视觉丢失目标五帧内
				{					
					vision.lost_cnt ++ ;   //丢失目标,帧数自加
					vision_calcu();			   //丢失目标5帧内仍计算视觉
				}
				else if(vision.lost_cnt >= 5)		//不在视觉模式或视觉丢失目标
				{
					ctrl_mode = KEYBOARD_MODE;	//视觉与键盘模式的切换，不用break
				}
			}	

			case KEYBOARD_MODE:
			{
				//gimbal.pid.pit_relative_ecd   = rc.mouse.y *  0.7f;
				//gimbal.pid.yaw_relative_angle = rc.mouse.x * -0.05f;
				 gimbal.pid.pit_relative_ecd = 0;
				 gimbal.pid.yaw_relative_angle = 0;
				normal_calcu();
				break;
			}
			
			default:
			{
			}break;
		}		

		memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
		osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
		
		taskEXIT_CRITICAL();     
		
		osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
	}
}

void normal_calcu()
{		
	gimbal.pid.pit_ecd_ref += gimbal.pid.pit_relative_ecd;				
	gimbal.pid.yaw_angle_ref += gimbal.pid.yaw_relative_angle;
	gimbal_pid_calcu();
}


void vision_calcu()
{
/*------------------------pit轴视觉目标值计算------------------------*/		
	vision.pit.kal.angle_input = vision.pit.angle_error[4] * 3.0f;	//卡尔曼角度输入值
	vision.pit.kal.speed_input = vision.pit.aim_speed * 3.0f;			//卡尔曼速度输入值  pit轴是目标相对速度
	pit_kf_result = Kalman2Filter_calc(&kalman2_pit_filter,vision.pit.kal.angle_input,vision.pit.kal.speed_input);		
	vision.pit.kal.angle_output = pit_kf_result[0];
	vision.pit.kal.speed_output = pit_kf_result[1];			//二阶卡尔曼滤波，滤波出目标位置与目标速度
	
	gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb + (vision.pit.kal.angle_output * 22.7527f) ;	//算出视觉目标值
	
/*------------------------yaw轴视觉目标值计算------------------------*/	
	
//	if(vision.yaw.kal.angle_input > 0)
//	{
//		if(vision_yaw_perdict < 0 )		vision_yaw_perdict = 0;
//	}
//	else if(vision.yaw.kal.angle_input < 0)
//	{
//		if(vision_yaw_perdict > 0 )		vision_yaw_perdict = 0;
//	}
	vision.yaw.kal.angle_input = vision.yaw.angle_error[4] * 5.0f + vision_yaw_perdict;	
	//卡尔曼角度输入值 放大了五倍 并加上预测量
	vision.yaw.kal.speed_input = vision.yaw.abs_speed * 5.0f;		//卡尔曼速度输入值 yaw轴为目标绝对速度
	yaw_kf_result = Kalman2Filter_calc(&kalman2_yaw_filter,vision.yaw.kal.angle_input,vision.yaw.kal.speed_input);
	vision.yaw.kal.angle_output = yaw_kf_result[0];
	vision.yaw.kal.speed_output = yaw_kf_result[1];			//二阶卡尔曼滤波，滤波出目标位置与目标速度
	
	vision_yaw_perdict = 0.1f * vision.yaw.kal.speed_output;   //yaw轴预测量为上一帧目标绝对速度
//	vision_yaw_kal_perdict = Kalman1Filter_calc(&kalman_perdict_yaw,vision_yaw_perdict);
	
	gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb - vision.yaw.kal.angle_output;
	gimbal_pid_calcu();
}

void gimbal_pid_calcu()			
{
/*------------------------pit轴串级pid计算------------------------*/	
	gimbal.pid.pit_ecd_ref  = data_limit(gimbal.pid.pit_ecd_ref,GIMBAL_PIT_MAX,GIMBAL_PIT_MIN);	//目标值限幅
	gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
	gimbal.pid.pit_ecd_error = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);
	pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_error);
	gimbal.pid.pit_spd_ref =  pid_pit_ecd.pos_out;   //PID外环目标值
	gimbal.pid.pit_spd_fdb =    imu_data.wy;					 //pit角速度反馈传进PID结构体
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
	
/*------------------------yaw轴串级pid计算------------------------*/
	if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;	
	else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//目标值限幅
	//gimbal.pid.yaw_angle_fdb=moto_yaw.ecd*360/8191;
	gimbal.pid.yaw_angle_fdb = imu_data.yaw;
	gimbal.pid.yaw_angle_error = circle_error(gimbal.pid.yaw_angle_ref,gimbal.pid.yaw_angle_fdb,360);
	pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_error);
	gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;			
	gimbal.pid.yaw_spd_fdb = imu_data.wz;		
	pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);	
	
	gimbal.current[0] = -1.0f * pid_yaw_spd.pos_out;
	gimbal.current[1] = -1.0f * pid_pit_spd.pos_out;
}
