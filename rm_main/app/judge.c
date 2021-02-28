#include "judge.h"
#include "string.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"


#if JUDGE_VERSION == JUDGE_2020
/*****************���ݽṹ�嶨��**********************/
frame_header											FrameHeader;		//֡ͷ

ext_game_state_t									Game_State;
ext_game_result_t									Game_Result;
ext_game_robot_HP_t               RobotHP;
ext_dart_status_t									Dart_Status;
ext_event_data_t									Event_Data;
ext_supply_projectile_action_t		Supply_Projectile_Action;
ext_referee_warning_t							Referee_Warning;
ext_dart_remaining_time_t					Dart_Remaining_Time;
ext_game_robot_status_t						Game_Robot_Status;
ext_power_heat_data_t							Power_Heat_Data;
ext_game_robot_pos_t							Robot_Position;
ext_buff_t												Buff;
ext_aerial_robot_energy_t					Aerial_Robot_Energy;
ext_robot_hurt_t									Robot_Hurt;
ext_shoot_data_t									Shoot_Data;
ext_bullet_remaining_t						Bullet_Remaining;
ext_rfid_status_t									RFID_Status;
ext_dart_client_cmd_t							Dart_Client;
robot_interactive_receivedata_t		Comm_getdata;

/******************************************************/

/*****************�������ݽṹ�嶨��**********************/
ext_SendClient_t									Judgesend_Data;
ext_SendClientDelete_t						Delete_data;
ext_client_custom_character_t			Judgesend_strings_Data;
robot_interactive_data_t					Comm_senddata;
/******************************************************/

int Color;//��ǰ�����˵���Ӫ
uint8_t Robot_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_Client_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**
  * @brief  ��ȡ�������ݺ����������жϺ�����ֱ�ӵ��ý��ж�ȡ
  * @param  ��Ӧ���ڵĻ���������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
int Judge_Read_Data(uint8_t *ReadFromUsart)
{
	uint8_t  retval_tf = FALSE; //������ʵ�Ա�־λ,ÿ�ζ�ȡʱ��Ĭ��ΪFALSE
	uint16_t judge_length;			//�����ֽڳ���
	int      CmdID=0;				
	if(ReadFromUsart == NULL)
	{
		return -1;
	}
	
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);  //д��֡ͷ
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:     //0x0001
						memcpy(&Game_State, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:    //0x0002
						memcpy(&Game_Result, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_robot_HP:       //0x0003
						memcpy(&RobotHP, (ReadFromUsart + DATA), LEN_robot_HP);
					break;
					
					case ID_darts_status:		//0x0004
						memcpy(&Dart_Status, (ReadFromUsart + DATA), LEN_darts_status);
					break;
					
					case ID_event_data:    	 //0x0101
						memcpy(&Event_Data, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_action:   //0x0102
						memcpy(&Supply_Projectile_Action, (ReadFromUsart + DATA), LEN_supply_action);
					break;
					
					case ID_judge_warning:  	//0x0104
						memcpy(&Referee_Warning, (ReadFromUsart + DATA), LEN_judge_warning);
					break;
					
					case ID_dart_remaining_time:  //0x0105
						memcpy(&Dart_Remaining_Time, (ReadFromUsart + DATA), LEN_darts_remaining_tim);
					break;
					
					case ID_robot_status:     //0x0201
						memcpy(&Game_Robot_Status, (ReadFromUsart + DATA), LEN_robot_status);
					break;
					
					case ID_robot_power:      //0x0202
						memcpy(&Power_Heat_Data, (ReadFromUsart + DATA), LEN_robot_power);
					break;
					
					case ID_robot_position:   //0x0203
						memcpy(&Robot_Position, (ReadFromUsart + DATA), LEN_robot_position);
					break;
					
					case ID_robot_buff:      	//0x0204
						memcpy(&Buff, (ReadFromUsart + DATA), LEN_robot_buff);
					break;
					
					case ID_AerialRobotEnergy:   //0x0205
						memcpy(&Aerial_Robot_Energy, (ReadFromUsart + DATA), LEN_AerialRobotEnergy);
					break;
					
					case ID_robot_hurt:      		//0x0206
						memcpy(&Robot_Hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					
//						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
//						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&Shoot_Data, (ReadFromUsart + DATA), LEN_shoot_data);
					break;
					
					case ID_bullet_remaining:     //0x0208
						memcpy(&Bullet_Remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
					
					case ID_RFID_status:      		//0x0209
						memcpy(&RFID_Status, (ReadFromUsart + DATA), LEN_RFID_status);
					break;
					
					case ID_dart_client:      		//0x020A
						memcpy(&Dart_Client, (ReadFromUsart + DATA), LEN_dart_client);
					break;
					
//					case ID_robot_interract:      			//0x0207
//						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
//						JUDGE_ShootNumCount();//������ͳ
//					break;
					
					case ID_robot_interract:      		//0x0301
						memcpy(&Comm_getdata, (ReadFromUsart + DATA), LEN_comm_data);//�����ߵ�ID2 Ŀ������˵�ID2 ����ID2 ���ݶ�20
					break;
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(FrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(FrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	return retval_tf;
}


//***************************************************************************************************************************************

/**
	* @brief  �����Զ������ݵ����Կͻ���
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define send_max_len     150
unsigned char CliendTxBuffer[send_max_len];
uint8_t battery_rectangle[3]="rec";
uint8_t battery_energy_1[3]="en1";
uint8_t battery_energy_2[3]="en2";
uint8_t battery_energy_3[3]="en3";
uint8_t battery_energy_4[3]="en4";
graphic_data_struct_t rec_draw1_data;
graphic_data_struct_t enc1_data[2];
graphic_data_struct_t enc2_data[3];
graphic_data_struct_t enc3_data[4];
graphic_data_struct_t enc4_data[5];
//int a=1;
//void judge_send_task(void const *argu)
//{
//	uint32_t mode_wake_time = osKernelSysTick();
//	for(;;)
//	{
//		taskENTER_CRITICAL();
////		data_pack_imagine_customize_init();
////		if(a==1)
////		{
////			data_pack_imagine(DRAW_IMAGINE5,(uint8_t*)enc4_data,CLIENT_DRAW_5_GRAPHS_CMD_ID);
////		}else if(a==2)
////		{
////			data_pack_imagine(DRAW_IMAGINE5,(uint8_t*)enc3_data,CLIENT_DRAW_5_GRAPHS_CMD_ID);
////		}else if(a==3)
////		{
////			data_pack_imagine(DRAW_IMAGINE5,(uint8_t*)enc2_data,CLIENT_DRAW_5_GRAPHS_CMD_ID);
////		}else if(a==4)
////		{
////			data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)enc1_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
////		}else if(a==5)
////		{
////			data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&rec_draw1_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
////		}
//		uint8_t test[20]="HelloWorld";
//		data_pack_comm_data(101,CLIENT_INTERACTIVE_CMD_ID_TEST,test);
//		taskEXIT_CRITICAL();
//		osDelayUntil(&mode_wake_time, JUDGESEND_PERIOD);//����ϵͳ�в���������,���ڲ�����̫��	
//	}	
//}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
int determine_red_blue(void)
{
	Robot_Self_ID = Game_Robot_Status.robot_id;//��ȡ��ǰ������ID
	
	if(Game_Robot_Status.robot_id > 100)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}
/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = determine_red_blue();
	if(Color == BLUE)
	{
		Judge_Client_ID = 0x0164 + (Robot_Self_ID-100);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_Client_ID = 0x0100 + Robot_Self_ID;//����ͻ���ID
	}
}

/**
	* @brief  �Զ������ݴ������
  * @param  Judgedatalength data_length
	*					uint8_t* data_locate  ���ݶ�ָ��
	*					client_data_cmd_e date_type
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void data_pack_imagine(Judgedatalength data_length,uint8_t* data_locate,client_data_cmd_e date_type)
{
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	Judgesend_Data.txFrameHeader.SOF = 0xA5;
	Judgesend_Data.txFrameHeader.DataLength = data_length-9;
	Judgesend_Data.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//д��֡ͷ����
	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//д��֡ͷCRC8У����
	Judgesend_Data.CmdID = ID_robot_interract;	
	Judgesend_Data.FrameHeader.send_ID 	 = Robot_Self_ID;//�����ߵ�ID
	Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	Judgesend_Data.FrameHeader.data_cmd_id=date_type;

	//���д�����ݶ�
	memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 2);
	memcpy(CliendTxBuffer + 7, (uint8_t*)&Judgesend_Data.FrameHeader, 6);
	memcpy(CliendTxBuffer + 13, (uint8_t*)data_locate, data_length-15);//6?
	Append_CRC16_Check_Sum(CliendTxBuffer,data_length);//д�����ݶ�CRC16У����	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,data_length);
}

/**
	* @brief  �Զ���uiͼ�γ�ʼ��
  * @param  void
  * @retval void
  * @attention  �˴��Զ���ͼ��
  */
void data_pack_imagine_customize_init(void)
{
	memcpy(&enc4_data[0].graphic_name, battery_rectangle, 3); //����
	enc4_data[0].operate_tpye=1;
	enc4_data[0].graphic_tpye=1;
	enc4_data[0].layer=0;
	enc4_data[0].color=2;
	enc4_data[0].width=10;
	enc4_data[0].start_x=1700;
	enc4_data[0].start_y=700;
	enc4_data[0].end_x=1800;
	enc4_data[0].end_y=400;					//��ؾ������
	
	
	//����Ϊ���������ʾ
	memcpy(&enc4_data[1].graphic_name, battery_energy_1, 3); //����
	enc4_data[1].operate_tpye=1;
	enc4_data[1].graphic_tpye=0;
	enc4_data[1].layer=1;
	enc4_data[1].color=3;
	enc4_data[1].width=70;
	enc4_data[1].start_x=1710;
	enc4_data[1].start_y=438;
	enc4_data[1].end_x=1790;
	enc4_data[1].end_y=438;
	
	memcpy(&enc4_data[2].graphic_name, battery_energy_2, 3); 
	enc4_data[2].operate_tpye=1;
	enc4_data[2].graphic_tpye=0;
	enc4_data[2].layer=1;
	enc4_data[2].color=3;
	enc4_data[2].width=70;
	enc4_data[2].start_x=1710;
	enc4_data[2].start_y=513;
	enc4_data[2].end_x=1790;
	enc4_data[2].end_y=513;
	
	memcpy(&enc4_data[3].graphic_name, battery_energy_3, 3); 
	enc4_data[3].operate_tpye=1;
	enc4_data[3].graphic_tpye=0;
	enc4_data[3].layer=1;
	enc4_data[3].color=3;
	enc4_data[3].width=70;
	enc4_data[3].start_x=1710;
	enc4_data[3].start_y=588;
	enc4_data[3].end_x=1790;
	enc4_data[3].end_y=588;
	
	memcpy(&enc4_data[4].graphic_name, battery_energy_4, 3);
	enc4_data[4].operate_tpye=1;
	enc4_data[4].graphic_tpye=0;
	enc4_data[4].layer=1;
	enc4_data[4].color=3;
	enc4_data[4].width=70;
	enc4_data[4].start_x=1710;
	enc4_data[4].start_y=663;
	enc4_data[4].end_x=1790;
	enc4_data[4].end_y=663;
	
	memcpy(enc3_data, enc4_data,60); 
	memcpy(enc2_data, enc4_data,45); 
	memcpy(enc1_data, enc4_data,30); 
	memcpy(&rec_draw1_data, enc4_data,15);  
}


/**
  * @brief  �ͻ���ɾ��ͼ��
  * @param  ����ѡ��0: �ղ�����
	*										1: ɾ��ͼ�㣻
	*										2: ɾ�����У�
  * @param  ͼ��ѡ��0~9
  * @attention  
  */
void data_pack_delete(type_graphic_delete_e type,uint8_t layer)
{
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	Judgesend_Data.txFrameHeader.SOF = 0xA5;
	Judgesend_Data.txFrameHeader.DataLength = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.delete_data);
	Judgesend_Data.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//д��֡ͷ����
	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//д��֡ͷCRC8У����
	Judgesend_Data.CmdID = ID_robot_interract;	
	Judgesend_Data.FrameHeader.send_ID 	 = Robot_Self_ID;//�����ߵ�ID
	Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���	
	Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_DELETE_GRAPH_CMD_ID;
	Judgesend_Data.clientdata.delete_data.operate_tpye = type;
	Judgesend_Data.clientdata.delete_data.layer = layer;
	
	memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
	memcpy(CliendTxBuffer + 13, (uint8_t*)&Judgesend_Data.clientdata.delete_data, 2);
	Append_CRC16_Check_Sum(CliendTxBuffer,JUDGE_DELETE);//д�����ݶ�CRC16У����	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,JUDGE_DELETE);
}
/**
  * @brief  �ͻ��˻����ַ�
  * @param  �����С
  * @param  �ַ�����
  * @param  �������
  * @param  ���x����
  * @param  ���y����
  * @param  �ַ�
  * @attention  
  */
void data_pack_code(ext_client_custom_character_t code_date,uint8_t* CODE)
{
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	Judgesend_Data.txFrameHeader.SOF = 0xA5;
	Judgesend_Data.txFrameHeader.DataLength = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.code_data);
	Judgesend_Data.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//д��֡ͷ����
	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//д��֡ͷCRC8У����
	Judgesend_Data.CmdID = ID_robot_interract;	
	Judgesend_Data.FrameHeader.send_ID 	 = Robot_Self_ID;//�����ߵ�ID
	Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���	
	Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_WRITE_STRINGS_CMD_ID;
	

	
	//���д�����ݶ�
	memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
	memcpy(CliendTxBuffer + 13, (uint8_t*)&code_date.grapic_data_struct, 15);
	memcpy(CliendTxBuffer + 28, (uint8_t*)CODE, 30);
	Append_CRC16_Check_Sum(CliendTxBuffer,PRINTF_CODE);//д�����ݶ�CRC16У����	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,PRINTF_CODE);
}

/**
  * @brief  �����˽�������
  * @param  Ŀ�������id
  * @param  ��������id������0x0200~0x02ffѡȡ������ID�����ɲ������Զ���
  * @param  ��������?
  * @attention  
  */
void data_pack_comm_data(uint16_t Robot_Target_ID,client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID,uint8_t* comm_date)
{
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	Judgesend_Data.txFrameHeader.SOF = 0xA5;
	Judgesend_Data.txFrameHeader.DataLength = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.comm_senddata);
	Judgesend_Data.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//д��֡ͷ����
	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//д��֡ͷCRC8У����
	Judgesend_Data.CmdID = ID_robot_interract;	
	Judgesend_Data.FrameHeader.send_ID 	 = Robot_Self_ID;//�����ߵ�ID
	Judgesend_Data.FrameHeader.receiver_ID = Robot_Target_ID;//Ŀ������˵�ID
	Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_INTERACTIVE_CMD_ID;
		
	//���д�����ݶ�
	memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
	memcpy(CliendTxBuffer + 13, (uint8_t*)comm_date,LEN_comm_data);
	Append_CRC16_Check_Sum(CliendTxBuffer,ROBOT_COMM);//д�����ݶ�CRC16У����	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,ROBOT_COMM);
}

#endif
