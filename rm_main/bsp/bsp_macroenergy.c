#include "bsp_macroenergy.h"
energy_msg_t energy_msg;
int16_t define_pitch_position[8];
float define_pitch_ecd[7];
void macro_energy_calcu(int16_t yaw_error_set,int16_t pitch_error_set)
{
	uint8_t i;
	i = 0;
	while(i<7)
	{
		if(define_pitch_position[i]<=pitch_error_set&&pitch_error_set<define_pitch_position[i+1])
		{
			energy_msg.pitch_error_out = define_pitch_ecd[i];
			break;
		}
		else
		{
			i++;
		}
	}
}

void energy_init()
{
	/*******能量机关位置标定*********/
	define_pitch_position[0] = 0;
	define_pitch_position[1] = 1;
	define_pitch_position[2] = 2;
	define_pitch_position[3] = 3;
	define_pitch_position[4] = 4;
	define_pitch_position[5] = 5;
	define_pitch_position[6] = 6;
	define_pitch_position[7] = 7;	
	
	define_pitch_ecd[0] = 0;
	define_pitch_ecd[1] = 0;
	define_pitch_ecd[2] = 0;
	define_pitch_ecd[3] = 0;
	define_pitch_ecd[4] = 0;
	define_pitch_ecd[5] = 0;
	define_pitch_ecd[6] = 0;
}
