#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "crc.h"
#include "protocol.h"
#include "data_fifo.h"
#include "cmsis_os.h"
#include "bsp_usart.h"

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
 

extern UART_HandleTypeDef huart6;

taurus_end_info taurus_end;
uint8_t num[4]="123";

//实现RM协议的序列化过程
void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len)
{
		taurus_end.end1 = END1_SOF;
		taurus_end.end2 = END2_SOF;
    //TODO 定义至少128字节大小缓存数组
    static uint8_t send_buf[128];
    uint16_t index = 0;
    //TODO 定义帧头结构体
    frame_header_struct_t referee_send_header;
    
    //TODO 初始化对应帧头结构体
    referee_send_header.SOF = HEADER_SOF;
    referee_send_header.data_length = len;
    referee_send_header.seq++;
    
    //TODO 生成CRC8校验
    Append_CRC8_Check_Sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    
    memcpy(send_buf, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(send_buf + index, (void*)&cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);

    //TODO 填充数据包
    memcpy(send_buf + index, (void*)buf, len);
    index += len;

    //TODO 生成CRC16校验
    Append_CRC16_Check_Sum(send_buf, REF_HEADER_CRC_CMDID_LEN + len);
    index += sizeof(uint16_t);
			
		memcpy(send_buf + index,(void*)&taurus_end,sizeof(taurus_end_info));
		index += sizeof(taurus_end_info);
   // memcpy(send_buf,num,sizeof(num));
			HAL_UART_Transmit_DMA(&huart6, send_buf, sizeof(send_buf)); 
   
}





