/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "remote_control.h"
#include "VideoTransmitter.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI������Ҫ�Ļ�����״̬����
static referee_info_t *referee_recv_info;            // ���յ��Ĳ���ϵͳ����
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
uint8_t UI_Seq;          // ����ţ�������referee�ļ�ʹ��
extern uint8_t is_track; // �Ƿ�����
// @todo ��Ӧ��ʹ��ȫ�ֱ���

/**
 * @brief  �жϸ���ID��ѡ��ͻ���ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // idС��7�Ǻ�ɫ,����7����ɫ,0Ϊ��ɫ��1Ϊ��ɫ   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // ����ͻ���ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);                       // ģʽ�л����
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) __attribute__((used)); // �����ú�����ʵ��ģʽ�Զ��仯

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // ��ʼ������ϵͳ�Ĵ���,�����ز���ϵͳ��������ָ��
    Interactive_data = UI_data;                            // ��ȡUI������Ҫ�Ļ�����״̬����
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    if (rc_ctrl[TEMP].key[KEY_PRESS].v || video_ctrl[TEMP].key[KEY_PRESS].v)
        MyUIInit();
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // ���׼��
static Graph_Data_t UI_Energy[3];      // ����������
static String_Data_t UI_State_sta[7];  // ������״̬,��ֻ̬�軭һ��
static String_Data_t UI_State_dyn[7];  // ������״̬,��̬��add����change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // ���û�г�ʼ������ϵͳ��ֱ��ɾ��ui����
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // ����δ�յ�����ϵͳ����,�ȴ�һ��ʱ����ټ��

    DeterminRobotID();                                            // ȷ��uiҪ���͵���Ŀ��ͻ���
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // ���UI

    // ���Ʒ����׼��
    // �����޸�Ϊ��̬,����ʶ�������Ƿ�ʶ��
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 8, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 8, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);

    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // ���Ƴ���״̬��־ָʾ
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 800, "level:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 150, 850, "Bounce:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);
    // ���Ƴ���״̬��־����̬
    // ���ڳ�ʼ��ʱxxx_last_modeĬ��Ϊ0�����Դ˴���ӦUIҲӦ����Ϊ0ʱ��Ӧ��UI����ֹģʽ�����������޷���λflag������UI�޷�ˢ��
    // �ȼ���ʾ����̬
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 21, 2, 270, 800, "1");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "fast     ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "medium ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);

    // ���̹�����ʾ����̬
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // ��������
    UIRectangleDraw(&UI_Energy[0], "ss7", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // ���̹�����ʾ,��̬
    UIFloatDraw(&UI_Energy[1], "sd7", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // ��������ʼ״̬
    UILineDraw(&UI_Energy[2], "sd8", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

// �����ú�����ʵ��ģʽ�Զ��仯,���ڼ�������Ͳ���ϵͳ�Ƿ���������
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // �����ú�����ʵ��ģʽ�Զ��仯
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        // _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5f;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1:
    {
        // _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        // _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
        // _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
}

static char *UIGetLevel()
{
    switch (referee_recv_info->GameRobotState.robot_level)
    {
    case 1:
        return "1";
    case 2:
        return "2";
    case 3:
        return "3";
    case 4:
        return "4";
    case 5:
        return "5";
    case 6:
        return "6";
    case 7:
        return "7";
    case 8:
        return "8";
    case 9:
        return "9";
    case 10:
        return "10";
    }
    return "0";
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // level
    if (_Interactive_data->Referee_Interactive_Flag.level_flag == 1)
    {
        UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, UIGetLevel());
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.level_flag = 0;
    }
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_FAST:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "fast     ");
            break;
        case CHASSIS_MEDIUM:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "medium   ");
            // �˴�ע�������������⣬������ͬ���ܸ��ǵ�
            break;
        case CHASSIS_SLOW:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "slow     ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
            break;
        }
        case GIMBAL_FREE_MODE:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
            break;
        }
        case GIMBAL_GYRO_MODE:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
            break;
        }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    // loader
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        switch (_Interactive_data->loader_mode)
        {
        case LOAD_REVERSE:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "reverse");
            break;
        case LOAD_SLOW:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "slow   ");
            break;
        case LOAD_MEDIUM:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "medium ");
            break;
        case LOAD_FAST:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "fast   ");
            break;
        default:
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        UIFloatDraw(&UI_Energy[1], "sd7", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UILineDraw(&UI_Energy[2], "sd8", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    // is_tracking
    if (_Interactive_data->Referee_Interactive_Flag.tracking_flag == 1)
    {
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 8, _Interactive_data->is_tracking ? UI_Color_Pink : UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 8, _Interactive_data->is_tracking == 1 ? UI_Color_Pink : UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 0;
    }
}

/**
 * @brief  ģʽ�л����,ģʽ�����л�ʱ����flag��λ
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    if (_Interactive_data->loader_mode != _Interactive_data->loader_mode_last)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_mode_last = _Interactive_data->loader_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    _Interactive_data->level = referee_recv_info->GameRobotState.robot_level;
    if (_Interactive_data->level != _Interactive_data->level_last)
    {
        _Interactive_data->Referee_Interactive_Flag.level_flag = 1;
        _Interactive_data->level_last = _Interactive_data->level;
    }

    _Interactive_data->is_tracking = is_track;
    if (_Interactive_data->is_tracking != _Interactive_data->is_tracking_last)
    {
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 1;
        _Interactive_data->is_tracking_last = _Interactive_data->is_tracking;
    }
}
