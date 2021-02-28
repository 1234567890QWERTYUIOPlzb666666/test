/** 
  * @file remote_msg.h
  * @version 1.0
  * @date Nov,16th 2018
	*
  * @brief  
	*
  *	@author lin kr
  *
  */
#ifndef __REMOTE_MSG_H__
#define __REMOTE_MSG_H__

#include "stm32f4xx_hal.h"

/* remote control information structure */
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	int16_t ch5;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
  } kb;
} rc_info_t;


/* ----------------------- PC Key Definition-------------------------------- */ 
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0) 
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<7) 

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
};

#define DBUS_MAX_LEN 50
#define DBUS_BUFLEN  18

extern rc_info_t   rc;

void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
#endif
