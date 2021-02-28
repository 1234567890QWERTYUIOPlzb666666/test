/** 
  * @file     math_calcu.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��ѧ���㺯��
	*
  *	@author   Fatmouse
  *
  */
#include "math_calcu.h"
#include "remote_msg.h"
#include "math.h"
#include "data_processing.h"

ramp_function_source_t chassis_x_ramp;
ramp_function_source_t chassis_y_ramp;
ramp_function_source_t chassis_w_ramp;
ramp_function_source_t chassis_super_x_ramp;
ramp_function_source_t chassis_super_y_ramp;
/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
	ramp_source_type->max_value = max;
	ramp_source_type->min_value = min;
	ramp_source_type->frame_period = frame_period;

  ramp_source_type->input = input;

  ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

  if (ramp_source_type->out > ramp_source_type->max_value)
  {
    ramp_source_type->out = ramp_source_type->max_value;
  }
  else if (ramp_source_type->out < ramp_source_type->min_value)
  {
    ramp_source_type->out = ramp_source_type->min_value;
  }
}

void Slope_On(Slope_Struct *V)
{
	V->last_ticks = V->ticks;
	V->ticks = osKernelSysTick();
	if(V->real_target !=V->limit_target)
	{
		if(V->real_target < V->limit_target)//�Ӳ���
		{
			V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
			if(V->real_target > V->limit_target)//�޷�
			{
				V->real_target =  V->limit_target;
			}
		}	
		else if(V->real_target > V->limit_target)//������
		{
			V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
			if(V->real_target < V->limit_target)//�޷�
			{
				V->real_target =  (short)V->limit_target;
			}
		}
	}
}

/**
  * @brief          ð��������
  * @param[in]      �������飬�����С
  * @retval         void
  */
void Bubble_Sort(float *a,uint8_t n)
{
	float buf;
	for(uint8_t i=0;i<n;++i)
	{
		for(uint8_t j=0;j<n-1-i;++j)
		{
			if(a[j]<a[j+1])
			{
				buf=a[j];
				a[j] = a[j+1];
				a[j+1] = buf;
			}
		}
	}
}

/**
  * @brief          �����˲�����   ע��N2Ϊ�������ڴ�С
  * @author         
  * @param[in]      �˲�ǰ��ֵ���˲���������
  * @retval         �˲����ֵ
  */
float GildeAverageValueFilter(float NewValue,float *Data)
{
  float max,min;
  float sum;
  uint16_t i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N2-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N2-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}

/**
  * @brief          Sigmoid����   
  * @author         
  * @param[in]      �Ա���
  * @retval         ӳ���ĺ���ֵ
  */
float Sigmoid_function(float x)
{
	float y;
	float temp_x=ABS(x) - SIGMOID_MAX;			//��sigmoid��������ƽ���������ֵ
	y= 1 / (1 + pow(e,-temp_x));
	return y;
}

void abs_limit(float *a, float ABS_MAX,float offset){
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;}

/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		�������ݼ���ƫ��ֵ
	*@param[in] set �趨ֵ get����ֵ circle_para һȦ��ֵ
	*@note	���������£�ֱ�Ӽ����PID�е�ƫ��ֵ
*/
float circle_error(float set ,float get ,float circle_para)
{
	float error;
	if(set > get)
	{
		if(set - get> circle_para/2)
			error = set - get - circle_para;
		else
			error = set - get;
	}
	else if(set < get)
	{
		if(set - get<-1*circle_para/2)
			error = set - get +circle_para;
		else
			error = set - get;
	}
	else	error = 0;

	return error;
}

/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period,const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

float data_limit(float data , float max, float min)
{
	if(data >= max)					return max;
	else if(data <= min)		return min;
	else 										return data;
}
