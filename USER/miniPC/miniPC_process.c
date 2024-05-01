#include "miniPC_process.h"
#include "string.h"
#include "daemon.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
#include "CRC.h"

static Vision_Instance *vision_instance; // ���ں��Ӿ�ͨ�ŵĴ���ʵ��
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *vision_daemon_instance;
// ȫ�ֱ�����
extern uint16_t CRC_INIT;
float yaw_send = 0;
uint8_t is_tracking = 0;
/**
 * @brief �����Ӿ����������
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* ʹ��memcpy���ո�����С�� */
    recv->is_tracking = rx_buff[1];
    memcpy(&recv->yaw, &rx_buff[2], 4);
    memcpy(&recv->pitch, &rx_buff[6], 4);

    /* ����У��λ */
    memcpy(&recv->checksum, &rx_buff[10], 2);

    /* �Ӿ����ݴ��� */
    if (recv->yaw > 180.0f)
        yaw_send = recv->yaw - 360.0f;
    else
        yaw_send = recv->yaw;

    is_tracking = recv->is_tracking;
    recv->pitch = -recv->pitch + 180;
}

/**
 * @brief �ص�������ȷ��֡ͷ�����ڽ����Ӿ�����
 *
 */
static void DecodeVision(void)
{
    DaemonReload(vision_daemon_instance); // ι��
#ifdef VISION_USE_VCP
    if (vis_recv_buff[0] == vision_instance->recv_data->header)
    {
        // ��ȡ�Ӿ�����
        RecvProcess(vision_instance->recv_data, vis_recv_buff);
    }
#endif
#ifdef VISION_USE_UART
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header)
    {
        // ��ȡ�Ӿ�����
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);
    }
#endif
}

/**
 * @brief ���߻ص�����,����daemon.c�б�daemon task����
 * @attention ����HAL����������,���ڿ���DMA����֮��ͬʱ�����и��ʳ���__HAL_LOCK()���µ�����,ʹ���޷�
 *            ��������ж�.ͨ��daemon�ж����ݸ���,���µ��÷������������Խ��������.
 *
 * @param id vision_usart_instance�ĵ�ַ,�˴�û��.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_instance->usart);
#endif // !VISION_USE_UART
}

/**
 * @brief ���÷��͸��Ӿ���IMU����
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    vision_instance->send_data->yaw = yaw;
    vision_instance->send_data->pitch = pitch;
    vision_instance->send_data->roll = roll;
}

/**
 * @brief �������ݴ�����
 *
 * @param send ����������
 * @param tx_buff ���ͻ�����
 *
 */
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* ����֡ͷ��Ŀ����ɫ���Ƿ����õ����� */
    tx_buff[0] = send->header;
    tx_buff[1] = send->detect_color;
    tx_buff[2] = send->reset_tracker;
    tx_buff[3] = is_tracking;

    /* ʹ��memcpy���͸�����С�� */
    memcpy(&tx_buff[4], &send->roll, 4);
    memcpy(&tx_buff[8], &send->yaw, 4);
    memcpy(&tx_buff[12], &send->pitch, 4);

    /* ����У��λ */
    send->checksum = Get_CRC16_Check_Sum(&tx_buff[0], VISION_SEND_SIZE - 3u, CRC_INIT);
    memcpy(&tx_buff[16], &send->checksum, 2);
    memcpy(&tx_buff[18], &send->tail, 1);
}

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header = send_config->header;
    send_data->detect_color = send_config->detect_color;
    send_data->reset_tracker = send_config->reset_tracker;
    send_data->is_shoot = send_config->is_shoot;
    send_data->tail = send_config->tail;
    return send_data;
}

#ifdef VISION_USE_UART

/**
 * @brief ����ע��һ���Ӿ�ͨ��ģ��ʵ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    init_config->usart_config.module_callback = DecodeVision;

    vision_instance->usart = USARTRegister(&init_config->usart_config);
    vision_instance->recv_data = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data = VisionSendRegister(&init_config->send_config);
    // Ϊmaster processע��daemon,�����ж��Ӿ�ͨ���Ƿ�����
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // ����ʱ���õĻص�����,���������ڽ���
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return vision_instance->recv_data;
}

/**
 * @brief ���ͺ���
 *
 * @param send ����������
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}
#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

/**
 * @brief ����ע��һ���Ӿ�ͨ��ģ��ʵ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config)
{
    UNUSED(init_config); // ��Ϊ����������
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
    vision_instance->recv_data = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data = VisionSendRegister(&init_config->send_config);
    // Ϊmaster processע��daemon,�����ж��Ӿ�ͨ���Ƿ�����
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // ����ʱ���õĻص�����,���������ڽ���
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return vision_instance->recv_data;
}

/**
 * @brief ���ͺ���
 *
 * @param send ����������
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif