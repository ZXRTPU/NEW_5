#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// ��������ɫ
	uint16_t Robot_ID;			// ��������ID
	uint16_t Cilent_ID;			// �������˶�Ӧ�Ŀͻ���ID
	uint16_t Receiver_Robot_ID; // �����˳���ͨ��ʱ�����ߵ�ID������ͱ�������ͬ��ɫ
} referee_id_t;

// �˽ṹ���������ϵͳ���������Լ�UI����������˳���ͨ�ŵ������Ϣ
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // ���յ���֡ͷ��Ϣ
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	referee_warning_t GameWarning;						   // 0x0104
	dart_info_t DartInfo;								   // 0x0105
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	projectile_allowance_t ProjectileAllowance;			   // 0x0208
	rfid_status_t RFIDStatus;							   // 0x0209

	// �Զ��彻�����ݵĽ���
	Communicate_ReceiveData_t ReceiveData;

	uint8_t init_flag;

} referee_info_t;

// ģʽ�Ƿ��л���־λ��0Ϊδ�л���1Ϊ�л���static����Ĭ��Ϊ0
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t loader_flag : 1;
	uint32_t Power_flag : 1;
	uint32_t level_flag : 1;
	uint32_t tracking_flag : 1;
} Referee_Interactive_Flag_t;

// �˽ṹ�����UI����������˳���ͨ�ŵ���Ҫ�������ǲ���ϵͳ����
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// ΪUI�����Լ�������������
	chassis_mode_e chassis_mode;			 // ����ģʽ
	gimbal_mode_e gimbal_mode;				 // ��̨ģʽ
	shoot_mode_e shoot_mode;				 // ����ģʽ����
	friction_mode_e friction_mode;			 // Ħ���ֹر�
	lid_mode_e lid_mode;					 // ���ոǴ�
	loader_mode_e loader_mode;				 // ��Ƶѡ��
	Chassis_Power_Data_s Chassis_Power_Data; // ���ʿ���
	uint8_t level;							 // �ȼ���ʾ
	uint8_t is_tracking;					 // �Ӿ��Ƿ�ʶ��

	// ��һ�ε�ģʽ������flag�ж�
	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	loader_mode_e loader_mode_last;
	Chassis_Power_Data_s Chassis_last_Power_Data;
	uint8_t level_last;
	uint8_t is_tracking_last;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief ����ϵͳͨ�ų�ʼ��,�ú������ʼ������ϵͳ����,�����ж�
 *
 * @param referee_usart_handle ����handle,C��һ���ô���6
 * @return referee_info_t* ���ز���ϵͳ����������,��������/Ѫ��/״̬��
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI���ƺͽ������ķ��ͽӿ�,��UI��������Ͷ��ͨ�ź�������
 * @note �ڲ�������һ��ʵʱϵͳ����ʱ����,������Ϊ����ϵͳ����CMD��������λ10Hz
 *
 * @param send ���������׵�ַ
 * @param tx_len ���ͳ���
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H
