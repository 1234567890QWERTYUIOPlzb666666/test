/** 
  * @file supercap_task. h
  * @version 1.0
  * @date Mar,24th 2019
	*
  * @brief ≥¨º∂µÁ»›
	*
  *	@author linking
  *
  */

#ifndef __SUPERCAP_TASK_H__
#define __SUPERCAP_TASK_H__

#include "stm32f4xx_hal.h"

#define SUPERCAP_PERIOD 5

void supercap_task(void const *argu);
void supercap_init(void);
#endif
