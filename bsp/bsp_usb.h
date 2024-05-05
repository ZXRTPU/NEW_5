/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief �ṩ��usb vpc(virtal com port)�Ĳ����ӿ�,hid��msf���Ǻ������
 * @attention ��һ��usb�޸���usbd_cdc_if.c�е�CDC_Receive_FS����,��ʹ��cube���ɺ�ᱻ����.������Ҫ��usbcdciftemplate����һ���Լ���ģ��
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note ���⴮�ڵĲ�����/У��λ/����λ�ȶ�̬�ɱ�,ȡ������λ�����趨 */
/* ʹ��ʱ����Ҫ������Щ����(��Ϊ�ӻ�) */

uint8_t *USBInit(USB_Init_Config_s usb_conf); // bsp��ʼ��ʱ���û�����ö���豸

void USBTransmit(uint8_t *buffer, uint16_t len); // ͨ��usb��������