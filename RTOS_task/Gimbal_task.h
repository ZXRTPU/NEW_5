#ifndef GIMBAL_TASK_H__
#define GIMBAL_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"


// typedef struct
// {
//     motor_info_t motor_info;     // �����Ϣ�ṹ��
//     fp32 pid_parameter[3];       // ��̨�����pid����
//     fp32 pid_angle_parameter[3]; // ��̨�����pid����
//     pid_struct_t pid;            // ��̨�����pid�ṹ��
//     pid_struct_t pid_angle;      // ��̨�����pid�ṹ��
//     fp32 speed_target;           // ��̨�����Ŀ���ٶ�
//     fp32 angle_target;           // ��̨�����Ŀ��Ƕ�
//     fp32 init_angle;             // ��̨����ĳ�ʼ�Ƕ�
// } gimbal_t;

void Gimbal_task(void const * argument);

#endif // !