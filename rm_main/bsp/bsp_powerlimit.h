#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define 	POWERLIMIT_CNT		60
#define   SUPERCAP_POWER		200
typedef struct
{		
	float     judge_chassis_power;				//裁判系统反馈回来的底盘实时功率
	uint16_t  judge_chassis_power_buffer; //裁判系统反馈回来的底盘缓冲能量
	
	float     chassis_power;			//实际底盘功率 250Hz
	float     max_power;					//上限功率
	float		  power_buffer;				//当前缓存能量剩余值
	
	float     limit_kp;						//限制比例
	float   	limit_temp;					//限制倍数
	uint16_t  min_power_buffer;		//缓冲能量限制值
	uint8_t   cnt;								//功率信息自加位，判断信息的连续性
	uint16_t supercap_power;      //超级电容模组反馈回来的实时充电功率
	uint16_t supercap_energy;     //超级电容模组反馈回来的实时能量百分比
}powercontrol_t;

void Power_Control(int16_t * current);
void PowerControl_Init(void);
void PowerParam_Update(void);
void PowerControl_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data);

extern powercontrol_t powercontrol;
extern pid_t pid_powercontrol;
extern uint8_t limit_status;
extern uint8_t  MAX_POWER_JUDGE;
#endif
