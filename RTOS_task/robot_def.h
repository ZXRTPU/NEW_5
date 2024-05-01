/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // ������#pragma once����#ifndef ROBOT_DEF_H(header guard)

// #define REMOTE_LINK // ʹ��ң����,������·
#define VIDEO_LINK // ʹ��ͼ����·

#pragma pack(1) // 1�ֽڶ���
// ����ģʽ����
/**
 * @brief ���������޸�Ϊ��̨�������,�������õ���ȥ׷��̨,��̨�Ĺ����ȵ���С.
 *
 */
typedef enum
{
    CHASSIS_FAST = 0, // ����ת�ٿ�
    CHASSIS_MEDIUM,   // ����ת���е�
    CHASSIS_SLOW,     // ����ת����
} chassis_mode_e;

// ��̨ģʽ����
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // ����������
    GIMBAL_FREE_MODE,      // ��̨�����˶�ģʽ,������̷���(���̴�ʱӦΪNO_FOLLOW)����ֵΪ���total_angle;�ƺ����Ը�Ϊȫ����IMU����?
    GIMBAL_GYRO_MODE,      // ��̨�����Ƿ���ģʽ,����ֵΪ������pitch,total_yaw_angle,���̿���ΪС���ݺ͸���ģʽ
} gimbal_mode_e;

// ����ģʽ����
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // Ħ���ֹر�
    FRICTION_ON,      // Ħ���ֿ���
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // ���ոǴ�
    LID_CLOSE,    // ���ոǹر�
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0, // ֹͣ����
    LOAD_REVERSE,  // ��ת
    LOAD_SLOW,     // ����
    LOAD_MEDIUM,   // ����
    LOAD_FAST,     // ����
} loader_mode_e;

// ��������,�Ӳ���ϵͳ��ȡ,�Ƿ��б�Ҫ����?
typedef struct
{ // ���ʿ���
    float chassis_power_mx;
} Chassis_Power_Data_s;

#pragma pack()

///**
// * @brief �����˳�ʼ��,���ڿ���rtos֮ǰ����.��Ҳ��Ψһ��Ҫ����main�����ĺ���
// *
// */
//void RobotInit();

