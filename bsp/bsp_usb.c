/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb�ǵ���bsp,��˲�����ʵ��
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
// #include "bsp_log.h"
#include "bsp_dwt.h"

static uint8_t *bsp_usb_rx_buffer; // ���յ������ݻᱻ��������,buffer sizeΪ2048
// ע��usb�������ݰ�(Full speedģʽ��)���Ϊ64byte,�������ܻ���ֶ������

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb�������λ(ģ��β�)��usbd_conf.c�е�HAL_PCD_MspInit()��
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // ��ȡ��������ָ��
    // usb�Ľ��ջص������������ﱻ����,�������ݱ�����bsp_usb_rx_buffer��
    // LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len); // ����
}
