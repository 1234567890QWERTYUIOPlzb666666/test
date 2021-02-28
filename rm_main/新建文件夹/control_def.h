#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"

#define NO3_DEF //¸ü¸Ä±øÖÖ

#ifdef NO3_DEF
/***********shoot************/
#define COVER_START 800
#define COVER_END 1800
/***********chassis**********/
#define rc_ch2_scale 12
#define rc_ch1_scale 12
/***********gimbal***********/
#define rc_ch4_scale -0.008f
#define rc_ch3_scale -0.0003f

#define gimbal_pit_center_offset 6800
#define gimbal_yaw_center_offset 3375
#define GIMBAL_PIT_MAX           7100	
#define GIMBAL_PIT_MIN           6600	
/*********PID-Gimbal*********/

#define pid_pit_ecd_P 0.7f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 20.0f
#define pid_pit_spd_I 0.1f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 25.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 70.0f

#define pid_yaw_spd_P 90.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

#define pid_yaw_vision_P 0.0f
#define pid_yaw_vision_I 0.0f
#define pid_yaw_vision_D 0.0f

#define pid_pitch_vision_P 0.3f
#define pid_pitch_vision_I 0.00005f
#define pid_pitch_vision_D 0.0f
#endif
#endif
