/**
 * @file daemon.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   �ػ�����
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DAEMON_H
#define DAEMON_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MAX_NUM 32 // ֧�ֵ�����ػ���������

/* ģ�����ߴ�����ָ�� */
typedef void (*offline_callback)(void *);

typedef struct daemon_ins {
    uint16_t reload_count;     // ����ֵ
    offline_callback callback; // ���ߴ�����,��ģ������ʱ����

    uint16_t temp_count; // ��ǰֵ,��Ϊ��˵��ģ�����߻��쳣
    void *id;            // ģ��id,���ڱ�ʶģ��,��ʼ��ʱ����
} Daemon_Instance;

/* daemon��ʼ������ */
typedef struct
{
    uint16_t reload_count;     // ʵ��������appΨһ��Ҫ���õ�ֵ?
    uint16_t init_count;       // ���ߵȴ�ʱ��,��Щģ����Ҫ�յ����ص�ָ��Żᷴ������,��pc����Ҫ����ʱ��
    offline_callback callback; // �쳣������,��ģ�鷢���쳣ʱ�ᱻ����

    void *owner_id; // idȡӵ��daemon��ʵ���ĵ�ַ,��DJIMotorInstance*,cast��void*����
} Daemon_Init_Config_s;

/**
 * @brief ע��һ��daemonʵ��
 *
 * @param config ��ʼ������
 * @return DaemonInstance* ����ʵ��ָ��
 */
Daemon_Instance *DaemonRegister(Daemon_Init_Config_s *config);

/**
 * @brief ��ģ���յ��µ����ݻ������������ʱ,���øú�������temp_count,�൱��"ι��"
 *
 * @param daemon daemonʵ��ָ��
 */
void DaemonReload(Daemon_Instance *daemon);

/**
 * @brief ȷ��ģ���Ƿ�����
 *
 * @param daemon
 * @return uint8_t �������ҹ�������,����1;���򷵻���. ���������쳣���ͺ�����״̬�Ƚ����Ż�.
 */
uint8_t DaemonIsOnline(Daemon_Instance *daemon);

/**
 * @brief ����rtos��,���ÿ��daemonʵ����temp_count��Ƶ�ʽ��еݼ�����.
 *        ģ��ɹ��������ݻ�ɹ������������temp_count��ֵΪreload_count.
 *
 */
void DaemonTask(void);

#endif // DAEMON_H