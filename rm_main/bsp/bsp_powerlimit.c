#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_can.h"

powercontrol_t powercontrol = {0};
uint8_t supercap_status = 0;
/***********************************/
//功率控制要调的参数  最大功率   最小缓冲能量	
uint8_t   MAX_POWER_JUDGE = 80;			//裁判系统
#define 	BUFFER_MIN				10.0f		//预测缓冲能量小于该值则进行电流分配
/***********************************/

void Power_Control(int16_t * current)
{
	static uint8_t powerlimit_statu=0;		//限制状态位  0时为不限制   1位限制
	static uint8_t powerlimit_cnt=0;			//限制计数位	每次缓冲能量低于5后限制100ms 即50次功率

	if(powercontrol.power_buffer < BUFFER_MIN)	
	{		
		powerlimit_statu=1;			//缓冲能量小时限制状态位 置1
		powerlimit_cnt = 0;
		powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//计算限制比例
	}
	
	if(powerlimit_statu)
	{
		powerlimit_cnt++;
		for(uint8_t i=0;i<4;i++)
		{
			current[i] /= (ABS(powercontrol.limit_temp)+1.0f) ;
		}	
	}
	if(powerlimit_cnt >= POWERLIMIT_CNT)
	{
		powerlimit_statu = 0;		//限制状态位清零
		powerlimit_cnt = 0;			//限制计数位清零
	}
}

void PowerParam_Update(void)
{
	powercontrol.max_power = MAX_POWER_JUDGE;
	powercontrol.power_buffer -= ( powercontrol.chassis_power - powercontrol.max_power ) * 0.002f;
	if(powercontrol.power_buffer>=60)	 powercontrol.power_buffer = 60;		//缓冲能量更新
	
	if(powercontrol.max_power == 50)
	{
		chassis.wheel_max = 5000;
	}
	else if(powercontrol.max_power == 60)
	{
		chassis.wheel_max = 5500;
	}
	else if(powercontrol.max_power == 80)
	{
		chassis.wheel_max = 6000;					//最大轮速更新
	}
	else if(powercontrol.max_power == SUPERCAP_POWER)
	{
		chassis.wheel_max = 8900;
	}
		
}


void PowerControl_Init(void)
{
	powercontrol.max_power = MAX_POWER_JUDGE;
	powercontrol.limit_kp = 3.0f;
	powercontrol.min_power_buffer = BUFFER_MIN ;
	chassis.keyboard_input = 50.0f;
	powercontrol.power_buffer = 60.0f;
}

void SuperCap_Update(void)			//超级电容状态更新函数
{
	static uint8_t Shift_key_up = 1;		//Shift键松开位
	if(rc.kb.bit.SHIFT && Shift_key_up) 
	{
		Shift_key_up = 0;
		supercap_status = ~supercap_status;	
	}
	else if(!rc.kb.bit.SHIFT)	Shift_key_up = 1;
}

void PowerControl_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
	switch(can_id)
  {
		case CAN_POWER_ID:
		{
			memcpy(&powercontrol.chassis_power,CAN_Rx_data,4); 
			powercontrol.cnt = CAN_Rx_data[4];
		}
		case CAN_SUPERCAP_FDB_ID:
		{
			
		}
	}
}
