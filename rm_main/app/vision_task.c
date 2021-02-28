#include "vision_task.h"
#include "bsp_can.h"
#include "string.h"
#include "gimbal_task.h"
#include "math.h"
#include "bsp_T_imu.h"

#define ABS(x)		((x>0)? (x): (-x))

vision_msg_t vision;

void vision_data_handler(uint32_t can_id,uint8_t * vision_data)
{
	static uint32_t mode_wake_time,last_mode_wake_time;
	switch(can_id)
	{
		case CAN_VISION1_ID:
		{
/*------------------------接受视觉信息------------------------*/			
			memcpy(&vision.yaw.angle_error[4], vision_data,4);		
			memcpy(&vision.pit.angle_error[4],(vision_data+4),4);
			
			mode_wake_time = osKernelSysTick();
			vision.period = mode_wake_time - last_mode_wake_time;
			last_mode_wake_time = mode_wake_time;			//获取视觉周期
			
/*------------------------pit目标速度获取-------------------*/	
			if(vision.pit.angle_error[4] && vision.pit.angle_error[3])
			{
				vision.pit.aim_speed = (vision.pit.angle_error[4] - vision.pit.angle_error[3]) / vision.period * 1000.0f;	
			}
			else
			{
				vision.pit.aim_speed = 0 ;
			}		

/*------------------------yaw目标速度获取------------------------*/	
			if(vision.yaw.angle_error[4] && vision.yaw.angle_error[3])
			{
				vision.yaw.aim_speed = (vision.yaw.angle_error[4] - vision.yaw.angle_error[3]) / vision.period * 1000.0f;	
				vision.yaw.abs_speed = (-1.0f * imu_data.wz / 100.0f) + vision.yaw.aim_speed;
			}
			else
			{
				vision.yaw.aim_speed = 0;
				vision.yaw.abs_speed = 0;
			}
			
/*------------------------替换历史信息------------------------*/				
			for(uint8_t i=0;i<4;i++)
			{
				vision.pit.angle_error[i] = vision.pit.angle_error[i+1];	
				vision.yaw.angle_error[i] = vision.yaw.angle_error[i+1];	
			}
			break;
		}
		
		case CAN_VISION2_ID:
		{
			memcpy(&vision.distance,vision_data,4);
			memcpy(&vision.cnt,vision_data+4,1);
			memcpy(&vision.data_frame,vision_data+5,1);
			break;
		}
	}
}

void vision_read_data(uint8_t * Vision_Data)	//串口  暂用
{
	static uint32_t mode_wake_time,last_mode_wake_time;
/*------------------------接受视觉信息------------------------*/			
	memcpy(&vision.yaw.angle_error[4], Vision_Data,4);		
	memcpy(&vision.pit.angle_error[4],(Vision_Data+4),4);
	memcpy(&vision.distance,(Vision_Data+8),4);
	memcpy(&vision.cnt,(Vision_Data+12),1);
	memcpy(&vision.data_frame,(Vision_Data+13),1);
	
	mode_wake_time = osKernelSysTick();
	vision.period = mode_wake_time - last_mode_wake_time;
	last_mode_wake_time = mode_wake_time;			//获取视觉周期
	
/*------------------------pit目标速度获取-------------------*/	
	if(vision.pit.angle_error[4] && vision.pit.angle_error[3])
	{
		vision.pit.aim_speed = (vision.pit.angle_error[4] - vision.pit.angle_error[3]) / vision.period * 1000.0f;	
	}
	else
	{
		vision.pit.aim_speed = 0 ;
	}		

/*------------------------yaw目标速度获取------------------------*/	
	if(vision.yaw.angle_error[4] && vision.yaw.angle_error[3])
	{
		vision.yaw.aim_speed = (vision.yaw.angle_error[4] - vision.yaw.angle_error[3]) / vision.period * 1000.0f;	
		vision.yaw.abs_speed = (-1.0f * imu_data.wz / 100.0f) + vision.yaw.aim_speed;
		if(ABS(vision.yaw.aim_speed) >= 60)
		{
			vision.yaw.aim_speed = 0;
			vision.yaw.abs_speed = 0;
		}
	}
	else
	{
		vision.yaw.aim_speed = 0;
		vision.yaw.abs_speed = 0;
	}
	
/*------------------------替换历史信息------------------------*/				
	for(uint8_t i=0;i<4;i++)
	{
		vision.pit.angle_error[i] = vision.pit.angle_error[i+1];	
		vision.yaw.angle_error[i] = vision.yaw.angle_error[i+1];	
	}
}
