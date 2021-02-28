/** 
  * @file     math_calcu.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��ѧ���㺯��,б��������
	*
  *	@author   Fatmouse
  *
  */
#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define N2 100
#define PI 3.141593f
#define e  2.718282f
#define SIGMOID_PERIOD 0.133333f
#define SIGMOID_MAX    10

#define ABS(x)		((x>0)? (x): (-x))

typedef struct
{
	float change_scale;
	float real_target;
	float limit_target;
	TickType_t ticks;
	TickType_t last_ticks;
}Slope_Struct;

typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

void Slope_On(Slope_Struct *V);
typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

extern ramp_function_source_t chassis_x_ramp;
extern ramp_function_source_t chassis_y_ramp;
extern ramp_function_source_t chassis_w_ramp;
extern ramp_function_source_t chassis_super_x_ramp;
extern ramp_function_source_t chassis_super_y_ramp;
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void Bubble_Sort(float *a,uint8_t n);
float GildeAverageValueFilter(float NewValue,float *Data);
float Sigmoid_function(float x);
float circle_error(float set ,float get ,float circle_para);
void abs_limit(float *a, float ABS_MAX,float offset);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
float data_limit(float data , float max, float min);
#endif
