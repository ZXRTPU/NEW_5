#include "daemon.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "string.h"

/* ���ڱ������е�daemon instance */
static Daemon_Instance *daemon_instances[DAEMON_MAX_NUM];
static uint8_t idx; // ���ڼ�¼��ǰע���daemon����

Daemon_Instance *DaemonRegister(Daemon_Init_Config_s *config)
{
    Daemon_Instance *daemon_instance = (Daemon_Instance *)malloc(sizeof(Daemon_Instance));
    memset(daemon_instance, 0, sizeof(Daemon_Instance));

    daemon_instance->id           = config->owner_id;
    daemon_instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // Ĭ������ֵΪ100
    daemon_instance->callback     = config->callback;
    daemon_instance->temp_count   = config->init_count == 0 ? 100 : config->init_count; // Ĭ�����ߵȴ�ʱ��Ϊ100
    daemon_instance->temp_count   = config->reload_count;

    daemon_instances[idx++] = daemon_instance;

    return daemon_instance;
}

/**
 * @brief ��ģ���յ��µ����ݻ������������ʱ,���øú�������temp_count,�൱��"ι��"
 *
 * @param daemon daemonʵ��ָ��
 */
void DaemonReload(Daemon_Instance *daemon)
{
    daemon->temp_count = daemon->reload_count;
}

/**
 * @brief ȷ��ģ���Ƿ�����
 *
 * @param daemon
 * @return uint8_t �������ҹ�������,����1;���򷵻���. ���������쳣���ͺ�����״̬�Ƚ����Ż�.
 */
uint8_t DaemonIsOnline(Daemon_Instance *daemon)
{
    return daemon->temp_count > 0;
}

/**
 * @brief ����rtos��,���ÿ��daemonʵ����temp_count��Ƶ�ʽ��еݼ�����.
 *        ģ��ɹ��������ݻ�ɹ������������temp_count��ֵΪreload_count.
 *
 */
void DaemonTask(void)
{
    Daemon_Instance *daemon;
    for (uint8_t i = 0; i < idx; i++) {
        daemon = daemon_instances[i];
        if (daemon->temp_count > 0) // �������������ֵ,˵����һ��ι����û�г�ʱ,���������һ
            daemon->temp_count--;
        else if (daemon->callback != NULL) // ������˵����ʱ��,���ûص�����(����еĻ�)
            daemon->callback(daemon->id);
        // @todo ���Լ������������led����ʾ
    }
}