#include "status_task.h"
#include "tim.h"
#include "DataScope_DP.h"
#include "usart.h"

#define  CHASSIS_LED  GPIOD,GPIO_PIN_5
#define  DBUS_LED			GPIOA,GPIO_PIN_15
#define  GYRO_LED			GPIOD,GPIO_PIN_3
#define  T2006_LED		GPIOD,GPIO_PIN_6
#define  PIT_LED			GPIOD,GPIO_PIN_7
#define  CAP_LED			GPIOC,GPIO_PIN_10
#define  VISION_LED		GPIOE,GPIO_PIN_13
status_t status;
/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void const *argu)
{
		if(status.chassis_status[0] && status.chassis_status[1] 
		&& status.chassis_status[2] && status.chassis_status[3])	HAL_GPIO_WritePin(CHASSIS_LED,GPIO_PIN_SET);
		else		HAL_GPIO_TogglePin(CHASSIS_LED);		//无信息回传则LED闪烁
		
		if(status.gimbal_status[1])		HAL_GPIO_WritePin(PIT_LED,GPIO_PIN_SET);
		else		HAL_GPIO_TogglePin(PIT_LED);			
		
		if(status.gimbal_status[2])		HAL_GPIO_WritePin(T2006_LED,GPIO_PIN_SET);
		else		HAL_GPIO_TogglePin(T2006_LED);
		
		if(status.dbus_status)			HAL_GPIO_WritePin(DBUS_LED,GPIO_PIN_SET);
		else		HAL_GPIO_TogglePin(DBUS_LED);		
		
		if(status.gyro_status)			HAL_GPIO_WritePin(GYRO_LED,GPIO_PIN_SET);
		else		HAL_GPIO_TogglePin(GYRO_LED);		
		
		status_restore(); 
	HAL_GPIO_TogglePin(CHASSIS_LED);	
	osDelay(1000);
}

void status_init()
{

}

void status_restore()
{
	status.dbus_status = 0;
	status.gyro_status = 0;
	for(int i = 0;i<4;i++)
	{	
	  status.chassis_status[i] = 0;
	}
	status.gimbal_status[0] = 0;
	status.gimbal_status[1] = 0;
	status.gimbal_status[2] = 0;
}
