/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  ң����ģ�鶨��ͷ�ļ�
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

#pragma anon_unions

// ����ң�������ݶ�ȡ,ң����������һ����СΪ2������
#define LAST 1
#define TEMP 0

// ��ȡ��������
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// ������ֵ�Ƿ����
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // ��������ʱ��ֵ
#define RC_SW_MID ((uint16_t)3)  // �����м�ʱ��ֵ
#define RC_SW_DOWN ((uint16_t)2) // ��������ʱ��ֵ
// �����жϿ���״̬�ĺ�
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// ��Ӧkey[x][0~16],��ȡ��Ӧ�ļ�;����ͨ��key[KEY_PRESS][Key_W]��ȡW���Ƿ���,������Ϊλ���ɾ��
#define Key_W 0
#define Key_S 1
#define Key_A 2
#define Key_D 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// �����Ե�λ��ṹ��,���Լ������������ٶ�
typedef union
{
    struct // ���ڷ��ʼ���״̬
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // ����memcpy������Ҫ����ǿ������ת��
} Key_t;

// @todo ��ǰ�ṹ��Ƕ�׹���,��Ҫ�����Ż�
typedef struct
{
    struct
    {
        int16_t rocker_l_; // ��ˮƽ
        int16_t rocker_l1; // ����ֱ
        int16_t rocker_r_; // ��ˮƽ
        int16_t rocker_r1; // ����ֱ
        int16_t dial;      // ��߲���

        uint8_t switch_left;  // ��࿪��
        uint8_t switch_right; // �Ҳ࿪��
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
			  int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // ��Ϊλ���ļ�������,�ռ����8��,�ٶ�����16~��
                  // 0����������
                  // 1��ctrl+����
                  // 2��shift+����

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief ����ң����ҡ�˵�ֵ,����660����С��-660��ֵ����Ϊ����Чֵ,��0
 *
 */
void RectifyRCjoystick();

/**
 * @brief ��ʼ��ң����,�ú����Ὣң����ע�ᵽ����
 *
 * @attention ע�������ȷ�Ĵ���Ӳ��,ң������C����ʹ��USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief ���ң�����Ƿ�����,����δ��ʼ��Ҳ��Ϊ����
 *
 * @return uint8_t 1:���� 0:����
 */
uint8_t RemoteControlIsOnline();

/**
 * @brief ң�������ݽ���
 *
 * @param sbus_buf ����buffer
 */
void sbus_to_rc(const uint8_t *sbus_buf);


#endif
