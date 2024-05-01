#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "Exchange_task.h"
#include "drv_can.h"
#include "bsp_dwt.h"
#include <stdbool.h>
#include <pid.h>
shooter_t shooter; // ���������Ϣ�ṹ��
// ���0Ϊ���̵�������1Ϊ���ոǵ�������2��3ΪĦ���ֵ��
#define TRIGGER_SINGLE_ANGLE 1620 // 36*360/8

extern RC_ctrl_t rc_ctrl; // ң������Ϣ�ṹ��
bool is_angle_control = false;//����
float current_time = 0;
float last_time = 0;
bool flag=1;
static void Shooter_Inint();         // ��������ĳ�ʼ��
static void model_choice();          // ģʽѡ��
static void dial_control();          // ���̵������
static void friction_control();      // Ħ���ֵ������
static void bay_control();           // ���յ������
static void shooter_current_given(); // ������

// ����-������ת�̶��Ƕ�
static void trigger_single_angle_move();
static void shoot_reverse();

void Shoot_task(void const * argument)
{
    Shooter_Inint();
    for (;;)
    {
        model_choice();
        shooter_current_given();
        osDelay(1);
    }
}

// ��������ĳ�ʼ��
static void Shooter_Inint(void)
{
    // ��ʼ��pid����
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    shooter.pid_angle_value[0] = 10;
    shooter.pid_angle_value[1] = 0.05;
    shooter.pid_angle_value[2] = 500;
    // ��ʼ��pid�ṹ��
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 10000, 10000);
    pid_init(&shooter.pid_angle, shooter.pid_angle_value, 20000, 30000); // trigger_angle

    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 20000, 20000);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 10000, 10000);

    // ��ʼ���ٶ�Ŀ��
    shooter.dial_speed_target = 0;
    shooter.target_angle = shooter.motor_info[0].total_angle;

    shooter.friction_speed_target[0] = 0, shooter.friction_speed_target[1] = 0;//����Ħ�����ٶ�
    shooter.bay_speed_target = 0;
}

// ģʽѡ��
static void model_choice(void)
{
    // ȡ��ע�Ϳ�ʼ����
    bay_control();
    // ȡ��ע�Ϳ�ʼ����
    if (rc_ctrl.rc.s[1] == 3 || rc_ctrl.rc.s[1] == 1)
    {
        // ����
        friction_control();
        // dial_control();
        // �Ҳ����У��������
        if (rc_ctrl.rc.s[0] == 3)
        {
            // ������������
            if (flag)
            {
                // HAL_Delay(5000);
                is_angle_control = true;
                trigger_single_angle_move();
                flag=0;
            }
            
            // else if (z_flag)
            // {
            //     is_angle_control = false;
            //     shoot_reverse();
            // }
        }

        // �Ҳ����£�ң��������
        // else
        else if (rc_ctrl.rc.s[1] ==1)
        {
            dial_control();
            is_angle_control = false;
        }
        else if (rc_ctrl.rc.s[0] == 2)
        {
            flag=1;
        //     // �󲦸��ϣ��������
        //     if (rc_ctrl.rc.s[1] == 1)
        //     {
                // is_angle_control = false;
        //         // shoot_start();
        //         shooter.dial_speed_target = 2000;
        //     }
        //     else
        //     {
        //         // shoot_stop();
        //         shooter.dial_speed_target = 0;
        //     }
        }
    }
    else
    {
        shooter.dial_speed_target = 0;
        shooter.motor_info[0].set_current=0;
            shooter.bay_speed_target = 0;
        // ֹͣ
        shooter.friction_speed_target[0] = 0;
        shooter.friction_speed_target[1] = 0;
    }
}

// ���̵������
static void dial_control(void)
{
    if (rc_ctrl.rc.s[1] == 1)
    {
        LEDR_OFF();
        shooter.dial_speed_target = 2000;
    }
    else
    {
        shooter.dial_speed_target = 0;
    }

    // if (rc_ctrl.rc.s[0] == 3)
    // {
        // ������������
    // if (rc_ctrl.rc.s[0] == 3) // press_left
    // {
    //     is_angle_control = true;
    //     trigger_single_angle_move();
    //     HAL_Delay(5000);
    //     }
    //     else if (z_flag)
    //     {
    //         is_angle_control = false;
    //         shoot_reverse();
    //     }
    // }
}

// Ħ���ֵ������
static void friction_control(void)
{

    shooter.friction_speed_target[0] = -8000;
    shooter.friction_speed_target[1] = 8000;
}

// ���յ������
static void bay_control(void)
{
    // ����
    shooter.bay_speed_target = 0;
}

// ������
static void shooter_current_given(void)
{
    // shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // ���̵��
    if (is_angle_control)
        shooter.motor_info[0].set_current = pid_calc_trigger(&shooter.pid_angle, shooter.target_angle, shooter.motor_info[0].total_angle); // ���̵��
    else
        shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // ���̵��

    shooter.motor_info[1].set_current = pid_calc(&shooter.pid_bay, shooter.motor_info[1].rotor_speed, shooter.bay_speed_target);          // ���յ��
    shooter.motor_info[2].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[2].rotor_speed, shooter.friction_speed_target[0]); // Ħ���ֵ��
    shooter.motor_info[3].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[3].rotor_speed, shooter.friction_speed_target[1]); // Ħ���ֵ��
    set_motor_current_shoot(1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
    // set_curruent(MOTOR_3508_1, hcan1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
}

/*************������ת�̶��Ƕ�***********/
static void trigger_single_angle_move()
{
    current_time = DWT_GetTimeline_ms();
    // �ж����η���ʱ����������˫��
    if (current_time - last_time > 1000)
    {
        last_time = DWT_GetTimeline_ms();
        shooter.target_angle = shooter.motor_info[0].total_angle + TRIGGER_SINGLE_ANGLE;
    }
}
/*****************��ת******************/
static void shoot_reverse()
{
    shooter.dial_speed_target = 250;
}
