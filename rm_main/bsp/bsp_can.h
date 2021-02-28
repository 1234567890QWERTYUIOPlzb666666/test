/** 
  * @file     bap_can.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief  
  *
  * @author   Fatmouse
  *
  */
 
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef  __BSP_CAN_GLOBALS
#define __BSP_CAN_EXT
#else
#define __BSP_CAN_EXT extern
#endif
#include "can.h"
#include "comm_task.h"
#include "bsp_powerlimit.h"

/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
	
  CAN_YAW_MOTOR_ID     = 0x205,
  CAN_PIT_MOTOR_ID     = 0x206, 
	CAN_TRIGGER_MOTOR_ID = 0x207,
	
	CAN_POWER_ID			   = 0x003,	
	CAN_SUPERCAP_FDB_ID  = 0x004,
	
	CAN_VISION1_ID       = 0x010,
	CAN_VISION2_ID       = 0x011,
} can_msg_id_e;

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

__BSP_CAN_EXT moto_measure_t moto_chassis[4] ;
__BSP_CAN_EXT moto_measure_t moto_pit;
__BSP_CAN_EXT moto_measure_t moto_yaw;
__BSP_CAN_EXT moto_measure_t motor_trigger;

void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan , uint8_t * CAN_Rx_data);
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan);

void can_device_init(void);
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
