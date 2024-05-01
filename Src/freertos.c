/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ins_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId InstaskHandle;
osThreadId ChassistaskHandle;
osThreadId UItaskHandle;
osThreadId ExchangetaskHandle;
osThreadId GimbaltaskHandle;
osThreadId ShoottaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Ins_task_RTOS(void const * argument);
void Chassis_task(void const * argument);
void UI_task(void const * argument);
void Exchange_task(void const * argument);
void Gimbal_task(void const * argument);
void Shoot_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Instask */
  osThreadDef(Instask, Ins_task_RTOS, osPriorityNormal, 0, 1024);
  InstaskHandle = osThreadCreate(osThread(Instask), NULL);

  /* definition and creation of Chassistask */
  osThreadDef(Chassistask, Chassis_task, osPriorityRealtime, 0, 512);
  ChassistaskHandle = osThreadCreate(osThread(Chassistask), NULL);

  /* definition and creation of UItask */
  osThreadDef(UItask, UI_task, osPriorityRealtime, 0, 512);
  UItaskHandle = osThreadCreate(osThread(UItask), NULL);

  /* definition and creation of Exchangetask */
  osThreadDef(Exchangetask, Exchange_task, osPriorityNormal, 0, 128);
  ExchangetaskHandle = osThreadCreate(osThread(Exchangetask), NULL);

  /* definition and creation of Gimbaltask */
  osThreadDef(Gimbaltask, Gimbal_task, osPriorityRealtime, 0, 512);
  GimbaltaskHandle = osThreadCreate(osThread(Gimbaltask), NULL);

  /* definition and creation of Shoottask */
  osThreadDef(Shoottask, Shoot_task, osPriorityNormal, 0, 256);
  ShoottaskHandle = osThreadCreate(osThread(Shoottask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Ins_task_RTOS */
/**
* @brief Function implementing the Instask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_task_RTOS */
void Ins_task_RTOS(void const * argument)
{
  /* USER CODE BEGIN Ins_task_RTOS */
	INS_Init();
  /* Infinite loop */
  for(;;)
  {
		INS_Task();
    osDelay(1);
  }
  /* USER CODE END Ins_task_RTOS */
}

/* USER CODE BEGIN Header_Chassis_task */
/**
* @brief Function implementing the Chassistask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_task */
__weak void Chassis_task(void const * argument)
{
  /* USER CODE BEGIN Chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_task */
}

/* USER CODE BEGIN Header_UI_task */
/**
* @brief Function implementing the UItask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_task */
__weak void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_task */
}

/* USER CODE BEGIN Header_Exchange_task */
/**
* @brief Function implementing the Exchangetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Exchange_task */
__weak void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN Exchange_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Exchange_task */
}

/* USER CODE BEGIN Header_Gimbal_task */
/**
* @brief Function implementing the Gimbaltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_task */
__weak void Gimbal_task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_task */
}

/* USER CODE BEGIN Header_Shoot_task */
/**
* @brief Function implementing the Shoottask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_task */
__weak void Shoot_task(void const * argument)
{
  /* USER CODE BEGIN Shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
