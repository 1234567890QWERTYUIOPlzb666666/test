/** 
  * @file debug_task.h
  * @version 1.0
  * @date 1,21 2019
	*
  * @brief  µ˜ ‘”√
	*
  *	@author lzh
  *
  */
#ifndef __DEBUG_TASK_H__
#define __DEBUG_TASK_H__

#ifdef  __DEBUG_TASK_GLOBALS
#define __DEBUG_TASK_EXT
#else
#define __DEBUG_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"	
#define DEBUG_BUFLEN 8
#define DEBUG_MAX_LEN 8
void debug_task(void const *argu);

#endif

