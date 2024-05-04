//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"

//#include "robot_def.h"
//#include "Shoot_task.h"
//#include "Gimbal_task.h"
//#include "exchange.h"
//#include "daemon.h"
//#include "ins_task.h"
//#include "drv_can.h"
//#include "bsp_init.h"

//osThreadId Gimbal_taskHandle;
//osThreadId shoot_taskHandle;
//osThreadId insTaskHandle;
//osThreadId exchangeTaskHandle;
//osThreadId daemonTaskHandle;

//void OSTaskInit();
//void StartINSTASK(void const *argument);
//void StartDAEMONTASK(void const *argument);

///**
// * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函�?
// *
// */
//void RobotInit()
//{
//    // 关闭中断,防止在初始化过程中发生中�?
//    // 请不要在初始化过程中使用中断和延时函数！
//    // 若必�?,则只允许使用DWT_Delay()
//    __disable_irq();
//    BSPInit();
//    CAN1_Init();
//    CAN2_Init();
//    Shooter_Inint();
//    Gimbal_Init();
//    ExchangInit(); // 上下位机通信初始�?

//    OSTaskInit();
//    __enable_irq();
//}

///**
// * @brief 初始化机器人任务,所有持续运行的任务都在这里初始�?
// *
// */
//void OSTaskInit()
//{
//    osThreadDef(instask, StartINSTASK, osPriorityRealtime, 0, 1024);
//    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保�??1khz的频率执�??

//    osThreadDef(exchangeTask, exchange_task, osPriorityNormal, 0, 128);
//    exchangeTaskHandle = osThreadCreate(osThread(exchangeTask), NULL);

//    osThreadDef(GimbalTask, Gimbal_task, osPriorityRealtime, 0, 512);
//    Gimbal_taskHandle = osThreadCreate(osThread(GimbalTask), NULL);

//    osThreadDef(shootTask, Shoot_task, osPriorityNormal, 0, 256);
//    shoot_taskHandle = osThreadCreate(osThread(shootTask), NULL);

//    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
//    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);
//}

//__attribute__((noreturn)) void StartINSTASK(void const *argument)
//{
//    INS_Init(); // 确保BMI088被正确初始化.
//    for (;;)
//    {
//        // 1kHz
//        INS_Task();
//        osDelay(1);
//    }
//}

//__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
//{
//    for (;;)
//    {
//        // 100Hz
//        DaemonTask();
//        osDelay(10);
//    }
//}