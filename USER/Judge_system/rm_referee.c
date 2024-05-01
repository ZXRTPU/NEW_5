/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @author modified by wexhi
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "daemon.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // ����ϵͳ���ջ�������С

static USART_Instance *referee_usart_instance; // ����ϵͳ����ʵ��
static Daemon_Instance *referee_daemon;		   // ����ϵͳ�ػ�����
static referee_info_t referee_info;			   // ����ϵͳ����

/**
 * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
 * @param  buff: ��ȡ���Ĳ���ϵͳԭʼ����
 * @retval �Ƿ�������ж�������
 * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // ͳ��һ֡���ݳ���
	if (buff == NULL)	   // �����ݰ��������κδ���
		return;

	// д��֡ͷ����(5-byte),�����ж��Ƿ�ʼ�洢��������
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// �ж�֡ͷ����(0)�Ƿ�Ϊ0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// ֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// ͳ��һ֡���ݳ���(byte),����CR16У��
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// ֡βCRC16У��
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2��8λƴ��16λint
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// ��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				// ��8���ֽڿ�ʼ�������� data=7
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: // 0x0002
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: // 0x0003
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: // 0x0101
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_supply_projectile_action: // 0x0102
					memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
					break;
				case ID_game_warning: // 0x0104
					memcpy(&referee_info.GameWarning, (buff + DATA_Offset), LEN_game_warning);
					break;
				case ID_dart_info: // 0x0105
					memcpy(&referee_info.DartInfo, (buff + DATA_Offset), LEN_dart_info);
					break;
				case ID_game_robot_state: // 0x0201
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: // 0x0202
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: // 0x0203
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: // 0x0204
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_aerial_robot_energy: // 0x0205
					memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
					break;
				case ID_robot_hurt: // 0x0206
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: // 0x0207
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_projectile_allowance: // 0x0208
					memcpy(&referee_info.ProjectileAllowance, (buff + DATA_Offset), LEN_projectile_allowance);
					break;
				case ID_rfid_status: // 0x0209
					memcpy(&referee_info.RFIDStatus, (buff + DATA_Offset), LEN_rfid_status);
					break;
				// @todo ��demoδ����ڱ������ڡ��״�Ĳ���ϵͳ���ݽ���
				case ID_student_interactive: // 0x0301   syhtodo���մ���δ����
					memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
				}
			}
		}
		// �׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�Ӷ��ж�һ�����ݰ��Ƿ��ж�֡����
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // ���һ�����ݰ������˶�֡����,���ٴε��ý�������,ֱ���������ݰ��������
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

/*����ϵͳ���ڽ��ջص�����,�������� */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}

// ����ϵͳ��ʧ�ص�����,���³�ʼ������ϵͳ����
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
}

/* ����ϵͳͨ�ų�ʼ�� */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	if (!referee_info.init_flag)
		referee_info.init_flag = 1;
	else
		return &referee_info;

	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3sû���յ�����,����Ϊ��ʧ,�������ڽ���
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief ����ϵͳ���ݷ��ͺ���
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
