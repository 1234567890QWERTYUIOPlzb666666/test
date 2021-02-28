#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define 	POWERLIMIT_CNT		60
#define   SUPERCAP_POWER		200
typedef struct
{		
	float     judge_chassis_power;				//����ϵͳ���������ĵ���ʵʱ����
	uint16_t  judge_chassis_power_buffer; //����ϵͳ���������ĵ��̻�������
	
	float     chassis_power;			//ʵ�ʵ��̹��� 250Hz
	float     max_power;					//���޹���
	float		  power_buffer;				//��ǰ��������ʣ��ֵ
	
	float     limit_kp;						//���Ʊ���
	float   	limit_temp;					//���Ʊ���
	uint16_t  min_power_buffer;		//������������ֵ
	uint8_t   cnt;								//������Ϣ�Լ�λ���ж���Ϣ��������
	uint16_t supercap_power;      //��������ģ�鷴��������ʵʱ��繦��
	uint16_t supercap_energy;     //��������ģ�鷴��������ʵʱ�����ٷֱ�
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
