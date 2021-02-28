/** 
  * @file debug_task.c
  * @version 1.0
  * @date 1,21 2019
	*
  * @brief  ������
	*
  *	@author chen wb
  *
  */
#define __DEBUG_TASK_GLOBALS
#include "debug_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "pid.h"
#include "bsp_powerlimit.h"

//����˵����MiniBalance��λ��ͨ�����ڴ�ӡ���ݲ���
//����˵����ֱ�����������е��ô˺�����ע����ʱ���ٴ�ӡ����
//�����޷��� 
void DataWave(UART_HandleTypeDef* huart)
{
		//Channel��ѡ��ͨ����1-10��
		//���뵥���ȸ�����
	
		
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_fdb, 1 );
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref, 2 );
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_ecd_fdb, 3 ); 
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_ecd_ref, 4 );   
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_fdb, 5 );
//		DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_ref , 6 );
	
		DataScope_Get_Channel_Data(chassis.current_fdb[0]/1000.0*24.0, 1 );
		DataScope_Get_Channel_Data(chassis.current_fdb[1]/1000.0*24.0, 2 );
		DataScope_Get_Channel_Data(chassis.current_fdb[2]/1000.0*24.0, 3 ); 
		DataScope_Get_Channel_Data(chassis.current_fdb[3]/1000.0*24.0, 4 );   
		DataScope_Get_Channel_Data(chassis.war_sum, 5 );
		DataScope_Get_Channel_Data(pid_chassis_angle.get[0], 6 );
		DataScope_Get_Channel_Data(pid_chassis_angle.set[0], 7 );
	
		CK.Send_Count = DataScope_Data_Generate(7);//������Ҫ���͵����ݸ���
		if(huart == &huart1)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((USART1->SR&0X40)==0);  
			USART1->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
		else if(huart == &huart6)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((USART6->SR&0X40)==0);  
			USART6->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
}

void debug_task(void const *argu)
{
	for(;;)
	{
		DataWave(&huart6);
		osDelay(20);
	}
}

