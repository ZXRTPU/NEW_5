#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief ��ʼ������ϵͳ��������(UI�Ͷ��ͨ��)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief ��referee task֮ǰ����,�����freertos.c��
 * 
 */
void MyUIInit();

/**
 * @brief ����ϵͳ��������(UI�Ͷ��ͨ��)
 *
 */
void UITask();

#endif // REFEREE_H
