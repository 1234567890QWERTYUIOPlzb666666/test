#ifndef _BSP_MACROENERGY_H_
#define _BSP_MACROENERGY_H_

#include "stm32f4xx_hal.h"

typedef struct
{
  int16_t  yaw_error_set;
  int16_t  pitch_error_set;
	float 	 yaw_error_out;
	float    pitch_error_out;
	
} energy_msg_t;
extern energy_msg_t energy_msg;
void macro_energy_calcu(int16_t yaw_error_set,int16_t pitch_error_set);
#endif
