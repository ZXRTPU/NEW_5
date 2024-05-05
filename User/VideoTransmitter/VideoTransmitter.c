#include "VideoTransmitter.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "string.h"
#include "drv_can.h"

#define RE_RX_BUFFER_SIZE 255u // ����ϵͳ���ջ�������С

Video_ctrl_t video_ctrl[2]; // ���ڴ洢ͼ����·�Ŀ�������,[0]:��ǰ����TEMP,[1]:��һ�ε�����LAST.���ڰ����������º��л����ж�
static uint8_t is_init;
static uint8_t send_buff[8]; // �������ݻ�����
// ͼ��ӵ�еĴ���ʵ��,��Ϊͼ���ǵ���,��������ֻ��һ��,�Ͳ���װ��
static USART_Instance *video_usart_instance;
static Daemon_Instance *video_daemon_instance;

static void VideoDataContorl()
{
    if (video_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl������
        video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (video_ctrl[TEMP].key[KEY_PRESS].shift) // shift������
        video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = video_ctrl[TEMP].key[KEY_PRESS].keys,                   // ��ǰ�����Ƿ���
        key_last = video_ctrl[LAST].key[KEY_PRESS].keys,                       // ��һ�ΰ����Ƿ���
        key_with_ctrl = video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // ��ǰctrl��ϼ��Ƿ���
        key_with_shift = video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  ��ǰshift��ϼ��Ƿ���
        key_last_with_ctrl = video_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // ��һ��ctrl��ϼ��Ƿ���
        key_last_with_shift = video_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // ��һ��shift��ϼ��Ƿ���

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5λΪctrl��shift,ֱ������
            continue;
        // �����ǰ��������,��һ�ΰ���û�а���,��ctrl��shift��ϼ�û�а���,�򰴼����¼�����1(��⵽������)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // ��ǰctrl��ϼ�����,��һ��ctrl��ϼ�û�а���,��ctrl��ϼ����¼�����1(��⵽������)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // ��ǰshift��ϼ�����,��һ��shift��ϼ�û�а���,��shift��ϼ����¼�����1(��⵽������)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
    video_ctrl[LAST] = video_ctrl[TEMP];
}

/**
 * @brief ͼ�����ݽ�������
 *
 * @param buff ͼ������
 */
void VideoRead(uint8_t *buff)
{
    memcpy(&video_ctrl[TEMP].key_data, buff, 12);
    *(uint16_t *)&video_ctrl[TEMP].key[KEY_PRESS] = video_ctrl[TEMP].key_data.keyboard_value;
    VideoDataContorl();
}

/**
 * @brief  ͼ�����ݽ��ջص�����
 *
 */
static void VideoTransmitterCallback()
{
    DaemonReload(video_daemon_instance);
    VideoRead(video_usart_instance->recv_buff);
}

static void VideoTransmitterLostCallback()
{
    USARTServiceInit(video_usart_instance);
}

/**
 * @brief
 *
 * @param vedeo_usart_handle
 * @return Video_ctrl_t*
 */
Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle)
{
    if (is_init)
        return video_ctrl;
    USART_Init_Config_s conf;
    conf.module_callback = VideoTransmitterCallback;
    conf.usart_handle = video_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE;
    video_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VideoTransmitterLostCallback,
        .owner_id = video_usart_instance,
        .reload_count = 30, // 0.3s
    };
    video_daemon_instance = DaemonRegister(&daemon_conf);

    is_init = 1;
    return video_ctrl;
}