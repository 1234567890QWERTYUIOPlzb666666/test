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
//���ʿ���Ҫ���Ĳ���  �����   ��С��������	
uint8_t   MAX_POWER_JUDGE = 80;			//����ϵͳ
#define 	BUFFER_MIN				10.0f		//Ԥ�⻺������С�ڸ�ֵ����е�������
/***********************************/

void Power_Control(int16_t * current)
{
	static uint8_t powerlimit_statu=0;		//����״̬λ  0ʱΪ������   1λ����
	static uint8_t powerlimit_cnt=0;			//���Ƽ���λ	ÿ�λ�����������5������100ms ��50�ι���

	if(powercontrol.power_buffer < BUFFER_MIN)	
	{		
		powerlimit_statu=1;			//��������Сʱ����״̬λ ��1
		powerlimit_cnt = 0;
		powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//�������Ʊ���
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
		powerlimit_statu = 0;		//����״̬λ����
		powerlimit_cnt = 0;			//���Ƽ���λ����
	}
}

void PowerParam_Update(void)
{
	powercontrol.max_power = MAX_POWER_JUDGE;
	powercontrol.power_buffer -= ( powercontrol.chassis_power - powercontrol.max_power ) * 0.002f;
	if(powercontrol.power_buffer>=60)	 powercontrol.power_buffer = 60;		//������������
	
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
		chassis.wheel_max = 6000;					//������ٸ���
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

void SuperCap_Update(void)			//��������״̬���º���
{
	static uint8_t Shift_key_up = 1;		//Shift���ɿ�λ
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
