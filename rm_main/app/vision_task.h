#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#include "stdint.h"
#include "KalmanFilter.h"

#define VISION_YAW_OFFSET		0.2f		//��������ƫ��
#define VISION_PIT_OFFSET		0.05f		//��������ƫ��

typedef struct
{
	float angle_input;
	float angle_output;
	float speed_input;
	float speed_output;
}kal_t;

typedef struct 
{
	float angle_error[5];
	float aim_speed;
	float abs_speed;
	kal_t kal;
}vision_gimbal_t;

typedef struct 
{
	vision_gimbal_t  yaw;
	vision_gimbal_t  pit;
	
	float  distance;				//�Ӿ��ش��з�����
	
	char 	lost_cnt;		//��ʧĿ���Լ�λ���ж϶�֡
	int  	period;
	char  cnt;				//�Լ�λ����֤�Ӿ���Ϣ�ڳ����仯
	char  data_frame;	//֡βλ
}vision_msg_t;



void vision_data_handler(uint32_t can_id,uint8_t * vision_data);
void vision_read_data(uint8_t * Vision_Data);

extern Kalman1_param_t kalman_pit;
extern vision_msg_t vision;
#endif
