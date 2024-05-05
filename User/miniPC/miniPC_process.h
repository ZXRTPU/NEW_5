/**
 * @file miniPC_process.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   ���ڴ���miniPC�����ݣ����������ͷ���
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MINIPC_PROCESS_H
#define MINIPC_PROCESS_H

#include "stdint.h"
#include "bsp_usart.h"

#define VISION_RECV_HEADER 0xA5u // �Ӿ���������֡ͷ
#define VISION_SEND_HEADER 0x5Au // �Ӿ���������֡ͷ
#define VISION_SEND_TAIL 0xAAu   // �Ӿ���������֡β

#define VISION_RECV_SIZE 12u // ��ǰΪ�̶�ֵ,12�ֽ�
#define VISION_SEND_SIZE 19u

// #pragma pack(1) // 1�ֽڶ���

/* �Ƿ�׷�� */
typedef enum
{
    VISION_NO_SHOOTING = 0u,
    VISION_SHOOTING = 1u,
} VISION_SHOOTING_e;

/* �Ƿ�����׷�� */
typedef enum
{
    VISION_RESET_TRACKER_NO = 0u,
    VISION_RESET_TRACKER_YES = 1u,
} VISION_RESET_TRACKER_e;

/* Ŀ��ID */
typedef enum
{
    VISION_OUTPOST = 0u,
    VISION_GUARD = 6u,
    VISION_BASE = 7u,
} VISION_ID_e;

/* װ�װ����� */
typedef enum
{
    VISION_ARMORS_NUM_BALANCE = 2u,
    VISION_ARMORS_NUM_OUTPOST = 3u,
    VISION_ARMORS_NUM_NORMAL = 4u,
} VISION_ARMORS_NUM_e;

/* �з�װ�װ���ɫ */
typedef enum
{
    VISION_DETECT_COLOR_RED = 0u,
    VISION_DETECT_COLOR_BLUE = 1u,
} VISION_DETECT_COLOR_e;

/* �Ӿ�ͨ�ų�ʼ�����սṹ�� */
typedef struct
{
    uint8_t header; // ͷ֡У��λ
} Vision_Recv_Init_Config_s;

/* �Ӿ�ͨ�ų�ʼ�����ͽṹ�� */
typedef struct
{
    uint8_t header;        // ͷ֡У��λ
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t reset_tracker; // �Ƿ�����׷���� ��0 bool is_shoot; // �Ƿ�������ģʽ ���� 1
    uint8_t is_shoot;      // �Ƿ�������ģʽ ���� 1
    uint8_t tail;          // β֡У��λ
} Vision_Send_Init_Config_s;

/* �Ӿ�ʵ����ʼ�����ýṹ�� */
typedef struct
{
    Vision_Recv_Init_Config_s recv_config; // �������ݽṹ��
    Vision_Send_Init_Config_s send_config; // �������ݽṹ��
    USART_Init_Config_s usart_config;      // ����ʵ���ṹ��
} Vision_Init_Config_s;

/* minipc -> stm32 (���սṹ��) */
#pragma pack(1) // 1�ֽڶ���
typedef struct
{
    uint8_t header;
    uint8_t is_tracking;
    float yaw;
    float pitch;
    uint16_t checksum;
} Vision_Recv_s;

/* stm32 -> minipc (���ͽṹ��) */
typedef struct
{
    uint8_t header;
    uint8_t detect_color;  // 0-red 1-blue ��1
    uint8_t reset_tracker; // �Ƿ�����׷���� ��0
    uint8_t is_shoot;      // �Ƿ�������ģʽ ���� 1
    float roll;            // rad
    float yaw;             // rad
    float pitch;           //
    uint16_t checksum;     // crc16У��λ https://blog.csdn.net/ydyuse/article/details/105395368
    uint8_t tail;          // β֡У��λ
} Vision_Send_s;
#pragma pack() // ȡ��1�ֽڶ���
/* �Ӿ�ͨ��ģ��ʵ�� */
typedef struct
{
    Vision_Recv_s *recv_data; // �������ݽṹ��ָ��
    Vision_Send_s *send_data; // �������ݽṹ��ָ��
    USART_Instance *usart;    // ����ʵ��ָ��
} Vision_Instance;

// #pragma pack() // ȡ��1�ֽڶ���

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config);

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config);

/**
 * @brief ����ע��һ���Ӿ�ͨ��ģ��ʵ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config);

/**
 * @brief ���ͺ���
 *
 *
 */
void VisionSend();

/**
 * @brief ���÷��͸��Ӿ���IMU����
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // MINIPC_PROCESS_H