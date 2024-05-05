#ifndef VIDEO_CONTROL_H
#define VIDEO_CONTROL_H

#include "stdint.h"
#include "main.h"
#include "usart.h"
#include "remote_control.h"
#include "referee_protocol.h"

#pragma pack(1)

typedef struct
{
    xFrameHeader FrameHeader;  // ���յ���֡ͷ��Ϣ
    uint16_t CmdID;            // ������
    remote_control_t key_data; // ң��������

    Key_t key[3]; // ��Ϊλ���ļ�������,�ռ����8��,�ٶ�����16~��

    uint8_t key_count[3][16];
} Video_ctrl_t;

#pragma pack()

Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);

void VideoRead(uint8_t *buff);

#endif // !VIDEO_CONTROL_H