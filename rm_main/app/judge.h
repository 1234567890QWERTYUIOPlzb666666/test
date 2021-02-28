#ifndef _JUDGE_H
#define _JUDGE_H

#include "stdint.h"
#include "crc.h"

#define JUDGE_2020	2020
#define 	JUDGE_VERSION JUDGE_2020



#if JUDGE_VERSION == JUDGE_2020


#define   LEN_HEADER    5       //帧头长
#define   LEN_CMDID     2       //命令码长度
#define   LEN_TAIL      2	      //帧尾CRC16
//#define		LEN_COMMDATA	20			//机器人间交互数据数据段

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
//发送数据到客户端 周期为200ms
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

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

typedef enum
{
	ID_game_state		  =		0x0001,		//比赛状态				1Hz
	ID_game_result		=		0x0002,		//比赛结果				结束时发送
	ID_robot_HP				=		0x0003,		//机器人血量			1Hz
	ID_darts_status		=		0x0004,		//飞镖发射状态		飞镖发射时发送
	
	ID_event_data			=		0x0101,		//场地实践数据		1Hz
	ID_supply_action	=		0x0102,		//补给站动作标识	动作发生后发送
	ID_judge_warning	=		0x0104,		//裁判警告数据		警告发生后发送
	ID_dart_remaining_time =0x0105,	//飞镖发射倒计时	1Hz
	
	ID_robot_status		=		0x0201,		//机器人状态数据	10Hz
	ID_robot_power		=		0x0202,		//实时功率热量		50Hz
	ID_robot_position =		0x0203,		//机器人位置数据	10Hz
	ID_robot_buff			=		0x0204,		//机器人增益数据	1Hz
	ID_AerialRobotEnergy =	0x0205, //无人机能量			10Hz,只发送空中
	ID_robot_hurt			=		0x0206,		//伤害状态数据		伤害发生后发送
	ID_shoot_data			=		0x0207,		//实时设计数据		子弹发射后发送
	ID_bullet_remaining = 0x0208,   //弹丸剩余发射数	1Hz，仅空中
	ID_RFID_status		=		0x0209,		//机器人RFID状态	1Hz
	ID_dart_client    =   0x020A,		//飞镖机器人客户端指令数据 10Hz
	
	ID_robot_interract=		0x0301,		//机器人间交互数据	发送方触发发送
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
	LEN_comm_data				=		20,			//机器人间交互数据数据段
//	LEN_robot_interract	=		,		
}cmd_LEN;

//以下为具体数据结构体

/* 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。*/
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//1机甲大师赛，2机甲大师单项赛
	uint8_t game_progress : 4;		//0未开始比赛，1准备阶段，2自检阶段，3五秒倒计时，4对战中，5比赛解算中
	uint16_t stage_remain_time;		//当前阶段剩余时间，单位秒
} ext_game_state_t; 

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。*/
typedef __packed struct 
{ 
	uint8_t winner;								//0 平局 1 红方胜利 2 蓝方胜利
} ext_game_result_t; 

/* 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。*/
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

/* 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。*/
typedef __packed struct
{
	uint8_t dart_belong;							//1红方飞镖，2蓝方飞镖 
	uint16_t stage_remaining_time; 		//发射时的剩余比赛时间，单位秒
}ext_dart_status_t;

/* 场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
typedef __packed struct
{
	uint32_t event_type;
	/*
	bit 0-1：己方停机坪占领状态
  0 为无机器人占领；
	1 为空中机器人已占领但未停桨；
	2 为空中机器人已占领并停桨
	
	bit 2-3：己方能量机关状态：
	bit 2 为小能量机关激活状态，1 为已激活；
	bit 3 为大能量机关激活状态，1 为已激活；
	bit 4 己方基地虚拟护盾状态；1 为基地有虚拟护盾血量；0 为基地无虚拟护盾血量；
	
  bit 5 -31: 保留
	
	*/
}ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。*/
typedef __packed struct
{
	uint8_t supply_projectile_id; 		//补给站口 ID：1 ，2
	uint8_t supply_robot_id; 					//补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，蓝方为..
	uint8_t supply_projectile_step; 	//出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
	uint8_t supply_projectile_num;		//补弹数量：50 ，100 ，150 ，200
}ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。*/
typedef __packed struct
{
	uint8_t level; 						//警告等级
	uint8_t foul_robot_id;		//犯规机器人ID ：1 级以及 5 级警告时，机器人 ID 为 0 ，二三四级警告时，机器人 ID 为犯规机器人 ID 
}ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人*/
typedef __packed struct
{ 
	uint8_t dart_remaining_time;
}ext_dart_remaining_time_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。*/
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

/* 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。*/
typedef __packed struct
{
	uint16_t chassis_volt; 					//单位毫伏
	uint16_t chassis_current; 			//单位毫安
	float chassis_power;						//单位瓦 
	uint16_t chassis_power_buffer;	//单位焦耳，备注：飞坡根据规则增加至250J 
	uint16_t shooter_heat0; 
	uint16_t shooter_heat1; 
	uint16_t mobile_shooter_heat2;
}ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
	float x;		//单位米
	float y;
	float z;
	float yaw;	//枪口位置，单位度
}ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。*/
typedef __packed struct
{
 uint8_t power_rune_buff;
	/* 
	bit 0：机器人血量补血状态
	bit 1：枪口热量冷却加成
	bit 2：机器人防御加成
	bit 3：机器人攻击加成
	*/
}ext_buff_t;

/* 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{ 
	uint16_t energy_point;	//积累的能量点
	uint8_t attack_time;		//可攻击时间，单位 s，30s 递减至 0	
}ext_aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	/*
	bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
	bit 4-7：血量变化类型:
	         0x0 装甲伤害扣血；
	         0x1 模块掉线扣血；
	         0x2 超射速扣血；
	         0x3 超枪口热量扣血；
	         0x4 超底盘功率扣血；
	         0x5 装甲撞击扣血
	*/
}ext_robot_hurt_t;

/* 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。*/
typedef __packed struct
{
  uint8_t bullet_type;		//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
  uint8_t bullet_freq;		//子弹射频 单位 Hz
  float bullet_speed;			//子弹射速 单位 m/s
}ext_shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，
	 哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人。*/
typedef __packed struct
{
  uint16_t bullet_remaining_num;
}ext_bullet_remaining_t;

/* 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。*/
typedef __packed struct
{
  uint32_t rfid_status;
	/*
	bit 0：基地增益点 RFID 状态；
  bit 1：高地增益点 RFID 状态；
  bit 2：能量机关激活点 RFID 状态；
  bit 3：飞坡增益点 RFID 状态；
  bit 4：前哨岗增益点 RFID 状态；
  bit 5：资源岛增益点 RFID 状态；
  bit 6：补血点增益点 RFID 状态；
  bit 7：工程机器人补血卡 RFID 状态；
  bit 8-25：保留
  bit 26-31：人工智能挑战赛 F1-F6 RFID 状态；
  RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果.
	
	*/
}ext_rfid_status_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。*/
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

/* 机器人间交互数据：0x0200~0x02ff。发送范围：单一机器人。*/
typedef __packed struct 
{ 
	uint8_t data[LEN_comm_data+6];
} robot_interactive_receivedata_t;//机器人间交互数据


/* 
	交互数据接收信息：0x0301。
	包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的字节限则如下
	雷达站			5120 字节每秒
	英雄机器人	3720 字节每秒
	工程机器人	3720 字节每秒
	步兵机器人	3720 字节每秒
	哨兵机器人	3720 字节每秒
	空中机器人	3720 字节每秒
	
	机器人 ID：
	1,英雄(红)；
	2,工程(红)；
	3/4/5,步兵(红)；
	6,空中(红)；
	7,哨兵(红)；
	9,雷达站(红);
	101,英雄(蓝)；
	102,工程(蓝)；
	103/104/105,步兵(蓝)；
	106,空中(蓝)；
	107,哨兵(蓝)
	109,雷达站(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端(红) ；
	0x0102 ，工程操作手客户端 (红)；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端(红)； 
	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。 


*/

//*****************************************************************
/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;					//起始字节
	uint16_t DataLength;	//数据长度
	uint8_t  Seq;					//包序号
	uint8_t  CRC8;				//crc8校验
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
}Judgedatalength;//自定义数据发送总长度


typedef enum
{
	CLIENT_INTERACTIVE_CMD_ID_TEST    = 0x0200 , //交互数据，可在0x0200~0x02ff选取，具体ID含义由参赛队自定义
	CLIENT_DELETE_GRAPH_CMD_ID     		= 0x0100 , //客户端删除图形
	CLIENT_DRAW_1_GRAPHS_CMD_ID     	= 0x0101 , //客户端绘制1个图形
	CLIENT_DRAW_2_GRAPHS_CMD_ID    		= 0x0102 , //客户端绘制2个图形
	CLIENT_DRAW_5_GRAPHS_CMD_ID    		= 0x0103 , //客户端绘制5个图形
	CLIENT_DRAW_7_GRAPHS_CMD_ID    		= 0x0104 , //客户端绘制7个图形
	CLIENT_WRITE_STRINGS_CMD_ID    		= 0x0110 , //客户端绘制字符串图形
}client_data_cmd_e;//数据段的内容 ID

/* 交互数据接收信息帧头 */
typedef __packed struct 
{ 
	client_data_cmd_e data_cmd_id;    
	uint16_t 					send_ID;    
	uint16_t 					receiver_ID; 
} ext_judgesend_custom_header_t;


//*****************************************************************
//图形删除类型
typedef enum
{
	type_delete_nop   = 0,
	type_delete_layer = 1,
	type_delete_all   = 2,
}type_graphic_delete_e;

/*客户端删除图形*/
typedef __packed struct
{ 
	type_graphic_delete_e operate_tpye;
	uint8_t layer;
} ext_SendClientDelete_t;

/* 
	图形数据
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
} ext_client_custom_character_t;//客户端绘制字符

typedef __packed struct 
{ 
	uint8_t data[LEN_comm_data];
} robot_interactive_data_t;//机器人间交互数据

typedef __packed struct
{
	ext_SendClientDelete_t 					delete_data;//(2)
	graphic_data_struct_t						draw1_data;//客户端绘制1个图形(15)
	graphic_data_struct_t						draw2_data[2];//客户端绘制2个图形(30)
	graphic_data_struct_t						draw5_data[5];//客户端绘制5个图形(75)
	graphic_data_struct_t						draw7_data[7];//客户端绘制7个图形(105)
	ext_client_custom_character_t		code_data;//客户端绘制字符(45)
	robot_interactive_data_t				comm_senddata;//机器人间交互数据(LEN_comm_data)
}ext_clientdata_t;//数据段

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	frame_header   									txFrameHeader;//帧头5
	cmd_ID		 											CmdID;//命令码2
	ext_judgesend_custom_header_t		FrameHeader;//数据段头结构6
	ext_clientdata_t  							clientdata;//数据段
	uint16_t		 										FrameTail;//帧尾2
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

#endif //版本定义

#endif //头文件定义
