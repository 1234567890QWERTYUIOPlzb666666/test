#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "crc.h"
#include "protocol.h"
#include "cmsis_os.h"
#include "bsp_usart.h"


	
extern chassis_ctrl_info_t chassis_ctrl;//底盘控制


 
uint8_t Rx_Buf[256];

//uint8_t s_count = 0;

//RM协议解包控制结构体
unpack_data_t referee_unpack_obj;
//RM协议反序列化函数
void referee_unpack_fifo_data(void);

//RM协议序列化函数
void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len);

frame_header_struct_t referee_receive_header;
extern UART_HandleTypeDef huart6;


//RM协议解析函数，系统自动调用
void referee_task(void const * argument)
{
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)Rx_Buf, 128);
	
    while(1)
    {
      referee_unpack_fifo_data();
      osDelay(1);
    }
}


//RM协议反序列化
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;
	uint8_t flag = 1;
	uint16_t index2 = 0;
	
  while (flag)
  {
    byte = *(Rx_Buf+index2);
		
    switch(p_obj->unpack_step)
    {
      //查找帧头
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
					index2++;
        }
        else
        {
          index2++;
		
        }
      }break;
      
      //获取数据长度低字节
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
				index2++;
      }break;
      
      //获取数据长度高字节
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;
				index2++;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
					
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
					index2++;
        }
      }break;
    
      //记录协议包序列号
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
				index2++;
      }break;

      //校验帧头CRC8
      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
				index2++;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
						
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
						index2++;
						
          }
        }
      }break;  
      
      //校验整帧CRC16
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
					index2++;
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
           p_obj->index = 0;
						index2++;

          if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            //成功解到一个正确的信息包
            referee_data_solve(p_obj->protocol_packet);
						memset(Rx_Buf,0,sizeof(Rx_Buf));
          }
        }
      }break;

      //解包失败重新寻找帧头
      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
         p_obj->index = 0;
				 index2++;
      }break;
			
    }
		
			if(index2 >= 255)
					{
						index2 = 0;
						//p_obj->unpack_step = STEP_HEADER_SOF;
						//p_obj->index = 0;
						
					}
		osDelay(1);
  }
}

uint16_t referee_data_solve(uint8_t *frame)
{
    uint8_t index = 0;
    uint16_t cmd_id = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
   
	switch (cmd_id)
    {
        //接收控制码对应信息包
        case CHASSIS_CTRL_CMD_ID:
        {
            memcpy(&chassis_ctrl, frame + index, sizeof(chassis_ctrl_info_t));
						
            break;
        }
				case GIMBAL_CTRL_ID:
				{
						//memcpy(&summer_camp_info, frame + index, sizeof(summer_camp_info_t));
					  
						break;
				}
				case GAME_INFO_CMD_ID:
				{
					  //memcpy(&message_info, frame + index, sizeof(message_info_t));
						
						break;
				}
				
        default:
        {
            break;
        }
    }

    index += referee_receive_header.data_length + 2;
    return index;
}
