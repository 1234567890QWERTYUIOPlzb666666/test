#ifndef _JUDGE_H
#define _JUDGE_H

#include "stdint.h"
#include "crc.h"

#define JUDGE_2020	2020
#define 	JUDGE_VERSION JUDGE_2020



#if JUDGE_VERSION == JUDGE_2020


#define   LEN_HEADER    5       //֡ͷ��
#define   LEN_CMDID     2       //�����볤��
#define   LEN_TAIL      2	      //֡βCRC16
//#define		LEN_COMMDATA	20			//�����˼佻���������ݶ�

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
//�������ݵ��ͻ��� ����Ϊ200ms
#define JUDGESEND_PERIOD 200

typedef enum 
{
	BLUE,
	RED
}CAMP;

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

typedef enum
{
	ID_game_state		  =		0x0001,		//����״̬				1Hz
	ID_game_result		=		0x0002,		//�������				����ʱ����
	ID_robot_HP				=		0x0003,		//������Ѫ��			1Hz
	ID_darts_status		=		0x0004,		//���ڷ���״̬		���ڷ���ʱ����
	
	ID_event_data			=		0x0101,		//����ʵ������		1Hz
	ID_supply_action	=		0x0102,		//����վ������ʶ	������������
	ID_judge_warning	=		0x0104,		//���о�������		���淢������
	ID_dart_remaining_time =0x0105,	//���ڷ��䵹��ʱ	1Hz
	
	ID_robot_status		=		0x0201,		//������״̬����	10Hz
	ID_robot_power		=		0x0202,		//ʵʱ��������		50Hz
	ID_robot_position =		0x0203,		//������λ������	10Hz
	ID_robot_buff			=		0x0204,		//��������������	1Hz
	ID_AerialRobotEnergy =	0x0205, //���˻�����			10Hz,ֻ���Ϳ���
	ID_robot_hurt			=		0x0206,		//�˺�״̬����		�˺���������
	ID_shoot_data			=		0x0207,		//ʵʱ�������		�ӵ��������
	ID_bullet_remaining = 0x0208,   //����ʣ�෢����	1Hz��������
	ID_RFID_status		=		0x0209,		//������RFID״̬	1Hz
	ID_dart_client    =   0x020A,		//���ڻ����˿ͻ���ָ������ 10Hz
	
	ID_robot_interract=		0x0301,		//�����˼佻������	���ͷ���������
}cmd_ID;

typedef enum
{
	LEN_game_state			=		3,		
	LEN_game_result			=		1,		
	LEN_robot_HP				=		32,		
	LEN_darts_status		=		3,		
	LEN_event_data			=		4,		
	LEN_supply_action		=		4,		
	LEN_judge_warning		=		2,		
	LEN_darts_remaining_tim	=		1,		
	LEN_robot_status		=		18,		
	LEN_robot_power			=		16,		
	LEN_robot_position 	=		16,		
	LEN_robot_buff			=   1,
	LEN_AerialRobotEnergy	=	3,
	LEN_robot_hurt			=		1,		
	LEN_shoot_data			=		6,		
	LEN_bullet_remaining=		2,		
	LEN_RFID_status			=		4,	
	LEN_dart_client			=   12,
	LEN_comm_data				=		20,			//�����˼佻���������ݶ�
//	LEN_robot_interract	=		,		
}cmd_LEN;

//����Ϊ�������ݽṹ��

/* ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//1���״�ʦ����2���״�ʦ������
	uint8_t game_progress : 4;		//0δ��ʼ������1׼���׶Σ�2�Լ�׶Σ�3���뵹��ʱ��4��ս�У�5����������
	uint16_t stage_remain_time;		//��ǰ�׶�ʣ��ʱ�䣬��λ��
} ext_game_state_t; 

/* ����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct 
{ 
	uint8_t winner;								//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��
} ext_game_result_t; 

/* ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP;
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}ext_game_robot_HP_t;

/* ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
	uint8_t dart_belong;							//1�췽���ڣ�2�������� 
	uint16_t stage_remaining_time; 		//����ʱ��ʣ�����ʱ�䣬��λ��
}ext_dart_status_t;

/* �����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint32_t event_type;
	/*
	bit 0-1������ͣ��ƺռ��״̬
  0 Ϊ�޻�����ռ�죻
	1 Ϊ���л�������ռ�쵫δͣ����
	2 Ϊ���л�������ռ�첢ͣ��
	
	bit 2-3��������������״̬��
	bit 2 ΪС�������ؼ���״̬��1 Ϊ�Ѽ��
	bit 3 Ϊ���������ؼ���״̬��1 Ϊ�Ѽ��
	bit 4 �����������⻤��״̬��1 Ϊ���������⻤��Ѫ����0 Ϊ���������⻤��Ѫ����
	
  bit 5 -31: ����
	
	*/
}ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint8_t supply_projectile_id; 		//����վ�� ID��1 ��2
	uint8_t supply_robot_id; 					//���������� ID��0 Ϊ��ǰ�޻����˲�����1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲���������Ϊ..
	uint8_t supply_projectile_step; 	//�����ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ�����
	uint8_t supply_projectile_num;		//����������50 ��100 ��150 ��200
}ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint8_t level; 						//����ȼ�
	uint8_t foul_robot_id;		//���������ID ��1 ���Լ� 5 ������ʱ�������� ID Ϊ 0 �������ļ�����ʱ�������� ID Ϊ��������� ID 
}ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������*/
typedef __packed struct
{ 
	uint8_t dart_remaining_time;
}ext_dart_remaining_time_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_heat0_cooling_rate;
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate;
	uint16_t shooter_heat1_cooling_limit;
	uint8_t shooter_heat0_speed_limit;
	uint8_t shooter_heat1_speed_limit;
	uint8_t max_chassis_power;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
}ext_game_robot_status_t;

/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint16_t chassis_volt; 					//��λ����
	uint16_t chassis_current; 			//��λ����
	float chassis_power;						//��λ�� 
	uint16_t chassis_power_buffer;	//��λ��������ע�����¸��ݹ���������250J 
	uint16_t shooter_heat0; 
	uint16_t shooter_heat1; 
	uint16_t mobile_shooter_heat2;
}ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	float x;		//��λ��
	float y;
	float z;
	float yaw;	//ǹ��λ�ã���λ��
}ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
 uint8_t power_rune_buff;
	/* 
	bit 0��������Ѫ����Ѫ״̬
	bit 1��ǹ��������ȴ�ӳ�
	bit 2�������˷����ӳ�
	bit 3�������˹����ӳ�
	*/
}ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{ 
	uint16_t energy_point;	//���۵�������
	uint8_t attack_time;		//�ɹ���ʱ�䣬��λ s��30s �ݼ��� 0	
}ext_aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	/*
	bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��
	bit 4-7��Ѫ���仯����:
	         0x0 װ���˺���Ѫ��
	         0x1 ģ����߿�Ѫ��
	         0x2 �����ٿ�Ѫ��
	         0x3 ��ǹ��������Ѫ��
	         0x4 �����̹��ʿ�Ѫ��
	         0x5 װ��ײ����Ѫ
	*/
}ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
  uint8_t bullet_type;		//�ӵ�����: 1��17mm ���� 2��42mm ����
  uint8_t bullet_freq;		//�ӵ���Ƶ ��λ Hz
  float bullet_speed;			//�ӵ����� ��λ m/s
}ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�1Hz ���ڷ��ͣ����л����ˣ�
	 �ڱ��������Լ� ICRA ���������ط��ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
  uint16_t bullet_remaining_num;
}ext_bullet_remaining_t;

/* ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
  uint32_t rfid_status;
	/*
	bit 0����������� RFID ״̬��
  bit 1���ߵ������ RFID ״̬��
  bit 2���������ؼ���� RFID ״̬��
  bit 3����������� RFID ״̬��
  bit 4��ǰ�ڸ������ RFID ״̬��
  bit 5����Դ������� RFID ״̬��
  bit 6����Ѫ������� RFID ״̬��
  bit 7�����̻����˲�Ѫ�� RFID ״̬��
  bit 8-25������
  bit 26-31���˹�������ս�� F1-F6 RFID ״̬��
  RFID ״̬����ȫ�����Ӧ������򴦷�״̬������з���ռ��ĸߵ�����㣬���ܻ�ȡ��Ӧ������Ч��.
	
	*/
}ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
  uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

/* �����˼佻�����ݣ�0x0200~0x02ff�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct 
{ 
	uint8_t data[LEN_comm_data+6];
} robot_interactive_receivedata_t;//�����˼佻������


/* 
	�������ݽ�����Ϣ��0x0301��
	����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 ���ֽ���������
	�״�վ			5120 �ֽ�ÿ��
	Ӣ�ۻ�����	3720 �ֽ�ÿ��
	���̻�����	3720 �ֽ�ÿ��
	����������	3720 �ֽ�ÿ��
	�ڱ�������	3720 �ֽ�ÿ��
	���л�����	3720 �ֽ�ÿ��
	
	������ ID��
	1,Ӣ��(��)��
	2,����(��)��
	3/4/5,����(��)��
	6,����(��)��
	7,�ڱ�(��)��
	9,�״�վ(��);
	101,Ӣ��(��)��
	102,����(��)��
	103/104/105,����(��)��
	106,����(��)��
	107,�ڱ�(��)
	109,�״�վ(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���(��) ��
	0x0102 �����̲����ֿͻ��� (��)��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���(��)�� 
	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)�� 


*/

//*****************************************************************
/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;					//��ʼ�ֽ�
	uint16_t DataLength;	//���ݳ���
	uint8_t  Seq;					//�����
	uint8_t  CRC8;				//crc8У��
} frame_header;

typedef enum 
{
	DRAW_IMAGINE1=30,
	DRAW_IMAGINE2=45,
	DRAW_IMAGINE5=90,
	DRAW_IMAGINE7=120,
	PRINTF_CODE	=60,
	JUDGE_DELETE=17,
	ROBOT_COMM=(15+LEN_comm_data),
}Judgedatalength;//�Զ������ݷ����ܳ���


typedef enum
{
	CLIENT_INTERACTIVE_CMD_ID_TEST    = 0x0200 , //�������ݣ�����0x0200~0x02ffѡȡ������ID�����ɲ������Զ���
	CLIENT_DELETE_GRAPH_CMD_ID     		= 0x0100 , //�ͻ���ɾ��ͼ��
	CLIENT_DRAW_1_GRAPHS_CMD_ID     	= 0x0101 , //�ͻ��˻���1��ͼ��
	CLIENT_DRAW_2_GRAPHS_CMD_ID    		= 0x0102 , //�ͻ��˻���2��ͼ��
	CLIENT_DRAW_5_GRAPHS_CMD_ID    		= 0x0103 , //�ͻ��˻���5��ͼ��
	CLIENT_DRAW_7_GRAPHS_CMD_ID    		= 0x0104 , //�ͻ��˻���7��ͼ��
	CLIENT_WRITE_STRINGS_CMD_ID    		= 0x0110 , //�ͻ��˻����ַ���ͼ��
}client_data_cmd_e;//���ݶε����� ID

/* �������ݽ�����Ϣ֡ͷ */
typedef __packed struct 
{ 
	client_data_cmd_e data_cmd_id;    
	uint16_t 					send_ID;    
	uint16_t 					receiver_ID; 
} ext_judgesend_custom_header_t;


//*****************************************************************
//ͼ��ɾ������
typedef enum
{
	type_delete_nop   = 0,
	type_delete_layer = 1,
	type_delete_all   = 2,
}type_graphic_delete_e;

/*�ͻ���ɾ��ͼ��*/
typedef __packed struct
{ 
	type_graphic_delete_e operate_tpye;
	uint8_t layer;
} ext_SendClientDelete_t;

/* 
	ͼ������
*/
typedef __packed struct 
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10; 
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct 
{
	graphic_data_struct_t grapic_data_struct; 
	uint8_t data[30];
} ext_client_custom_character_t;//�ͻ��˻����ַ�

typedef __packed struct 
{ 
	uint8_t data[LEN_comm_data];
} robot_interactive_data_t;//�����˼佻������

typedef __packed struct
{
	ext_SendClientDelete_t 					delete_data;//(2)
	graphic_data_struct_t						draw1_data;//�ͻ��˻���1��ͼ��(15)
	graphic_data_struct_t						draw2_data[2];//�ͻ��˻���2��ͼ��(30)
	graphic_data_struct_t						draw5_data[5];//�ͻ��˻���5��ͼ��(75)
	graphic_data_struct_t						draw7_data[7];//�ͻ��˻���7��ͼ��(105)
	ext_client_custom_character_t		code_data;//�ͻ��˻����ַ�(45)
	robot_interactive_data_t				comm_senddata;//�����˼佻������(LEN_comm_data)
}ext_clientdata_t;//���ݶ�

//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	frame_header   									txFrameHeader;//֡ͷ5
	cmd_ID		 											CmdID;//������2
	ext_judgesend_custom_header_t		FrameHeader;//���ݶ�ͷ�ṹ6
	ext_clientdata_t  							clientdata;//���ݶ�
	uint16_t		 										FrameTail;//֡β2
}ext_SendClient_t;

extern ext_power_heat_data_t	Power_Heat_Data;

int Judge_Read_Data(uint8_t *ReadFromUsart);
void judge_send_task(void const *argu);

int determine_red_blue(void);
void determine_ID(void);
void data_pack_imagine_customize_init(void);

void data_pack_imagine(Judgedatalength data_length,uint8_t* data_locate,client_data_cmd_e date_type);
void data_pack_code(ext_client_custom_character_t code_date,uint8_t* CODE);
void data_pack_delete(type_graphic_delete_e type,uint8_t layer);
void data_pack_comm_data(uint16_t Robot_Target_ID,client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID,uint8_t* comm_date);

#endif //�汾����

#endif //ͷ�ļ�����
