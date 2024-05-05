#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "drv_can.h"
#include "stdlib.h"
#include "daemon.h"
#include "struct_typedef.h"
#include "stdbool.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // ң�������յ�buffer��С

// ����ȫ�ֱ���
extern bool vision_is_tracking;
extern uint8_t friction_flag;

int16_t gimbal_yaw = 0; // ���ڽ���yaw��ֵ
int16_t gimbal_pitch = 0;

// ң��������
RC_ctrl_t rc_ctrl[2];            //[0]:��ǰ����TEMP,[1]:��һ�ε�����LAST.���ڰ����������º��л����ж�
static uint8_t rc_init_flag = 0; // ң������ʼ����־λ
static uint8_t temp_remote[8];   // ��ʱ�洢��������

// ң����ӵ�еĴ���ʵ��,��Ϊң�����ǵ���,��������ֻ��һ��,�Ͳ���װ��
static USART_Instance *rc_usart_instance;
static Daemon_Instance *rc_daemon_instance;

/**
 * @brief ����ң����ҡ�˵�ֵ,����660����С��-660��ֵ����Ϊ����Чֵ,��0
 *
 */
void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief ң�������ݽ���
 *
 * @param sbus_buf ����buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // ҡ��,ֱ�ӽ���ʱ��ȥƫ��
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                                       //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = ((((sbus_buf[1] >> 3) & 0xff) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = ((((sbus_buf[2] >> 6) & 0xff) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = ((((sbus_buf[4] >> 1) & 0xff) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                          // ��ದ��
    RectifyRCjoystick();
    // ����,0��1��
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // ������
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
	  rc_ctrl[TEMP].mouse.z = (sbus_buf[10]| (sbus_buf[11] << 8));	
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?
	  
	  // 0����������  1��ctrl+���� 2��shift+����
	  rc_ctrl[TEMP].key[0].v = sbus_buf[14] | (sbus_buf[15] << 8);	
//		w_flag = (sbus_buf[14] & 0x01);
//		s_flag = (sbus_buf[14] & 0x02);
//		a_flag = (sbus_buf[14] & 0x04);
//		d_flag = (sbus_buf[14] & 0x08);
//		q_flag = (sbus_buf[14] & 0x40);
//		e_flag = (sbus_buf[14] & 0x80);
//		shift_flag = (sbus_buf[14] & 0x10);
//		ctrl_flag = (sbus_buf[14] & 0x20);
//		press_left =  rc_ctrl[TEMP].mouse.press_l;
//		press_right =  rc_ctrl[TEMP].mouse.press_r;
//		// HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_11);
//		r_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x01 << 8);
//		f_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x02 << 8);
//		g_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x04 << 8);
//		z_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x08 << 8);
//		x_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x10 << 8);
//		c_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x20 << 8);
//		v_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x40 << 8);
//		b_flag = rc_ctrl[TEMP].key[0].v & (0x00 | 0x80 << 8);

    if (rc_ctrl[TEMP].rc.switch_left)
    {
        // CAN����ң�������ݸ���C��
        // ң��������
        for (int i = 0; i <= 7; i++)
        {
            temp_remote[i] = sbus_buf[i]; // volatile const uint8_t��uint8_t��һ������ֱ�Ӵ���can_remote�������
        }
        can_remote(temp_remote, 0x33,8);

        // ��������
        for (int i = 8; i <= 15; i++)
        {
            temp_remote[i - 8] = sbus_buf[i]; // volatile const uint8_t��uint8_t��һ������ֱ�Ӵ���can_remote�������
        }
        can_remote(temp_remote, 0x34,8);
    }


    // λ��İ���ֵ����,ֱ��memcpy����,ע��С�˵��ֽ���ǰ,��lsb�ڵ�һλ,msb�����
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl������
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift������
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // ��ǰ�����Ƿ���
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // ��һ�ΰ����Ƿ���
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // ��ǰctrl��ϼ��Ƿ���
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  ��ǰshift��ϼ��Ƿ���
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // ��һ��ctrl��ϼ��Ƿ���
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // ��һ��shift��ϼ��Ƿ���

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5λΪctrl��shift,ֱ������
            continue;
        // �����ǰ��������,��һ�ΰ���û�а���,��ctrl��shift��ϼ�û�а���,�򰴼����¼�����1(��⵽������)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // ��ǰctrl��ϼ�����,��һ��ctrl��ϼ�û�а���,��ctrl��ϼ����¼�����1(��⵽������)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // ��ǰshift��ϼ�����,��һ��shift��ϼ�û�а���,��shift��ϼ����¼�����1(��⵽������)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // ������һ�ε�����,���ڰ����������º��л����ж�
}

/**
 * @brief ��sbus_to_rc�ļ򵥷�װ,����ע�ᵽbsp_usart�Ļص�������
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // ��ι��
    sbus_to_rc(rc_usart_instance->recv_buff); // ����Э�����
}

/**
 * @brief ң�������ߵĻص�����,ע�ᵽ�ػ�������,���ڵ���ʱ����
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // ���ң��������
    USARTServiceInit(rc_usart_instance); // ����������������
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // �����ػ����̵�ע��,���ڶ�ʱ���ң�����Ƿ���������
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100msδ�յ�������Ϊ����,ң�����Ľ���Ƶ��ʵ������1000/14Hz(��Լ70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // ֻ��1��ң����,����Ҫowner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}