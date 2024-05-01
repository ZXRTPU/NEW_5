/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************�궨�岿��****************************/

#define REFEREE_SOF 0xA5 // ��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define Communicate_Data_LEN 5 // �Զ��彻�����ݳ��ȣ��ó��Ⱦ������ҷ����ͺ��������գ��Զ��彻������Э�����ʱֻ��Ҫ���Ĵ˺궨�弴��

#pragma pack(1)

/****************************ͨ��Э���ʽ****************************/

/* ͨ��Э���ʽƫ�ƣ�ö������,����#define���� */
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

/* ͨ��Э�鳤�� */
typedef enum
{
	LEN_HEADER = 5, // ֡ͷ��
	LEN_CMDID = 2,	// �����볤��
	LEN_TAIL = 2,	// ֡βCRC16

	LEN_CRC8 = 4, // ֡ͷCRC8У�鳤��=֡ͷ+���ݳ�+�����
} JudgeFrameLength_e;

/****************************֡ͷ****************************/
/****************************֡ͷ****************************/

/* ֡ͷƫ�� */
typedef enum
{
	SOF = 0,		 // ��ʼλ
	DATA_LENGTH = 1, // ֡�����ݳ���,�����������ȡ���ݳ���
	SEQ = 3,		 // �����
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* ֡ͷ���� */
typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id������˵��****************************/
/****************************cmd_id������˵��****************************/

/* ������ID,�����жϽ��յ���ʲô���� */
typedef enum
{
	ID_game_state = 0x0001,				   // ����״̬����
	ID_game_result = 0x0002,			   // �����������
	ID_game_robot_survivors = 0x0003,	   // ����������Ѫ������
	ID_event_data = 0x0101,				   // �����¼�����
	ID_supply_projectile_action = 0x0102,  // ���ز���վ������ʶ����
	ID_supply_projectile_booking = 0x0103, // ���ز���վԤԼ�ӵ�����
	ID_game_warning = 0x0104,			   // ���о�����Ϣ
	ID_dart_info = 0x0105,				   // ���ڷ�������
	ID_game_robot_state = 0x0201,		   // ������״̬����
	ID_power_heat_data = 0x0202,		   // ʵʱ������������
	ID_game_robot_pos = 0x0203,			   // ������λ������
	ID_buff_musk = 0x0204,				   // ��������������
	ID_aerial_robot_energy = 0x0205,	   // ���л���������״̬����
	ID_robot_hurt = 0x0206,				   // �˺�״̬����
	ID_shoot_data = 0x0207,				   // ʵʱ�������
	ID_projectile_allowance = 0x0208,	   // ������������
	ID_rfid_status = 0x0209,			   // RFID״̬����
	// ��demo��Ĭ��Ϊ���������ˣ����δ�����������ݵĽ���������Ҫ����
	// ���������
	ID_dart_client_cmd = 0x020A,	   // ����ѡ�ֶ�ָ������
	ID_ground_robot_position = 0x020B, // ���������λ������
	ID_radar_mark_data = 0x020C,	   // �״��ǽ�������
	ID_sentry_info = 0x020D,		   // �ڱ�����������Ϣͬ��
	ID_radar_info = 0x020E,			   // �״�����������Ϣͬ��
	ID_student_interactive = 0x0301,   // �����˼佻������
	// ����Ϊͼ����·���ݽ���
	ID_custom_robot_data = 0x0302,	 // �Զ������������
	ID_remote_control_data = 0x0304, // ͼ����·��������
} CmdID_e;

/* ���������ݶγ�,���ݹٷ�Э�������峤�ȣ������Զ������ݳ��� */
typedef enum
{
	LEN_game_state = 11,			  // 0x0001
	LEN_game_result = 1,			  // 0x0002
	LEN_game_robot_HP = 32,			  // 0x0003
	LEN_event_data = 4,				  // 0x0101
	LEN_supply_projectile_action = 4, // 0x0102
	LEN_game_warning = 3,			  // 0x0104
	LEN_dart_info = 3,				  // 0x0105
	LEN_game_robot_state = 13,		  // 0x0201
	LEN_power_heat_data = 16,		  // 0x0202
	LEN_game_robot_pos = 16,		  // 0x0203
	LEN_buff_musk = 6,				  // 0x0204
	LEN_aerial_robot_energy = 2,	  // 0x0205
	LEN_robot_hurt = 1,				  // 0x0206
	LEN_shoot_data = 7,				  // 0x0207
	LEN_projectile_allowance = 6,	  // 0x0208
	LEN_rfid_status = 4,			  // 0x0209
	// ��demo��Ĭ��Ϊ���������ˣ����δ�����������ݵĽ���������Ҫ����
	// ���������
	LEN_dart_client_cmd = 6,					 // 0x020A
	LEN_ground_robot_position = 40,				 // 0x020B
	LEN_radar_mark_data = 6,					 // 0x020C
	LEN_sentry_info = 4,						 // 0x020D
	LEN_radar_info = 1,							 // 0x020E
	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301
	// ����Ϊͼ����·���ݽ���
	LEN_custom_robot_data = 30,	  // 0x0302
	LEN_remote_control_data = 12, // 0x0304

} JudgeDataLength_e;

/****************************�������ݵ���ϸ˵��****************************/
/****************************�������ݵ���ϸ˵��****************************/

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    ����������� */
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef struct
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
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    �����¼����� */
typedef struct
{
	uint32_t event_type;
} ext_event_data_t;

/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ID: 0x0104  Byte:  3    ���о�������  */
typedef struct
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} referee_warning_t;

/* ID: 0x0105  Byte:  3    ���ڷ���������� */
typedef struct
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} dart_info_t;

/* ID: 0X0201  Byte: 13    ������״̬���� */
typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef struct
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;				 // ˲ʱ����
	uint16_t buffer_energy;				 // ��������
	uint16_t shooter_17mm_1_barrel_heat; // 17mmǹ��1����
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef struct
{
	float x;
	float y;
	float angle;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  6    �������������� */
typedef struct
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  2    ���л���������״̬���� */
typedef struct
{
	uint8_t airforce_status;
	uint8_t time_remain;
} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  6    ������������ */
typedef struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
} projectile_allowance_t;

/* ID: 0x0209  Byte:  4    RFID״̬���� */
typedef struct
{
	uint32_t rfid_status;
} rfid_status_t;

/* ID: 0x020A  Byte:  6    ����ѡ�ֶ�ָ������ */
typedef struct
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/* ID: 0x020B  Byte:  40    ���������λ������ */
typedef struct
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
} ground_robot_position_t;

/* ID: 0x020C  Byte:  6    �״��ǽ������� */
typedef struct
{
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
} radar_mark_data_t;

/* ID: 0x020D  Byte:  4    �ڱ�����������Ϣͬ�� */
typedef struct
{
	uint32_t sentry_info;
} sentry_info_t;

/* ID: 0x020E  Byte:  1    �״�����������Ϣͬ�� */
typedef struct
{
	uint8_t radar_info;
} radar_info_t;

/* ID: 0x0302   Byte: 30    �Զ��������������˽������� */
typedef struct
{
	uint8_t data[30];
} custom_robot_data_t;

/* ID: 0x0304   Byte: 12    ͼ����·����ң������ */
typedef struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control_t;

/****************************�����˽�������****************************/
/****************************�����˽�������****************************/
/* ���͵��������ݶ����Ϊ 113 ����Ƿ񳬳���С����?ʵ����ͼ�ζβ��ᳬ�����ݶ����30����Ҳ���ᳬ*/
/* ��������ͷ�ṹ */
typedef struct
{
	uint16_t data_cmd_id; // ���ڴ��ڶ������ ID��������cmd_id ����Ƶ�����Ϊ 10Hz��������Ŵ���ע�⽻�����ֵ�����Ƶ��
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* ������id */
typedef enum
{
	// �췽������ID
	RobotID_RHero = 1,
	RobotID_REngineer = 2,
	RobotID_RStandard1 = 3,
	RobotID_RStandard2 = 4,
	RobotID_RStandard3 = 5,
	RobotID_RAerial = 6,
	RobotID_RSentry = 7,
	RobotID_RRadar = 9,
	// ����������ID
	RobotID_BHero = 101,
	RobotID_BEngineer = 102,
	RobotID_BStandard1 = 103,
	RobotID_BStandard2 = 104,
	RobotID_BStandard3 = 105,
	RobotID_BAerial = 106,
	RobotID_BSentry = 107,
	RobotID_BRadar = 109,
} Robot_ID_e;

/* ��������ID */
typedef enum
{
	UI_Data_ID_Del = 0x100,
	UI_Data_ID_Draw1 = 0x101,
	UI_Data_ID_Draw2 = 0x102,
	UI_Data_ID_Draw5 = 0x103,
	UI_Data_ID_Draw7 = 0x104,
	UI_Data_ID_DrawChar = 0x110,

	/* �Զ��彻�����ݲ��� */
	Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;
/* �������ݳ��� */
typedef enum
{
	Interactive_Data_LEN_Head = 6,
	UI_Operate_LEN_Del = 2,
	UI_Operate_LEN_PerDraw = 15,
	UI_Operate_LEN_DrawChar = 15 + 30,

	/* �Զ��彻�����ݲ��� */
	// Communicate_Data_LEN = 5,

} Interactive_Data_Length_e;

/****************************�Զ��彻������****************************/
/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�Զ��彻������ �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz
*/
// �Զ��彻������Э�飬�ɸ��ģ����ĺ���Ҫ�޸����Ϸ��궨�����ݳ��ȵ�ֵ
typedef struct
{
	uint8_t data[Communicate_Data_LEN]; // ���ݶ�,n��ҪС��113
} robot_interactive_data_t;

// �����˽�����Ϣ_����
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // ���ݶ�
	uint16_t frametail;
} Communicate_SendData_t;
// �����˽�����Ϣ_����
typedef struct
{
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // ���ݶ�
} Communicate_ReceiveData_t;

/****************************UI��������****************************/

/* ͼ������ */
typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
} Graph_Data_t;

typedef struct
{
	Graph_Data_t Graph_Control;
	uint8_t show_Data[30];
} String_Data_t; // ��ӡ�ַ�������

/* ɾ������ */
typedef enum
{
	UI_Data_Del_NoOperate = 0,
	UI_Data_Del_Layer = 1,
	UI_Data_Del_ALL = 2, // ɾ��ȫ��ͼ�㣬����Ĳ����Ѿ�����Ҫ�ˡ�
} UI_Delete_Operate_e;

/* ͼ�����ò���__ͼ�β��� */
typedef enum
{
	UI_Graph_ADD = 1,
	UI_Graph_Change = 2,
	UI_Graph_Del = 3,
} UI_Graph_Operate_e;

/* ͼ�����ò���__ͼ������ */
typedef enum
{
	UI_Graph_Line = 0,		// ֱ��
	UI_Graph_Rectangle = 1, // ����
	UI_Graph_Circle = 2,	// ��Բ
	UI_Graph_Ellipse = 3,	// ��Բ
	UI_Graph_Arc = 4,		// Բ��
	UI_Graph_Float = 5,		// ������
	UI_Graph_Int = 6,		// ����
	UI_Graph_Char = 7,		// �ַ���

} UI_Graph_Type_e;

/* ͼ�����ò���__ͼ����ɫ */
typedef enum
{
	UI_Color_Main = 0, // ������ɫ
	UI_Color_Yellow = 1,
	UI_Color_Green = 2,
	UI_Color_Orange = 3,
	UI_Color_Purplish_red = 4, // �Ϻ�ɫ
	UI_Color_Pink = 5,
	UI_Color_Cyan = 6, // ��ɫ
	UI_Color_Black = 7,
	UI_Color_White = 8,

} UI_Graph_Color_e;

#pragma pack()

#endif
