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
#include <stdlib.h>
#include "cpp_link.hpp"
#include "lwip/apps/lwiperf.h"

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
osThreadId debugTaskHandle;
osThreadId ctrlSystemTaskHandle;
osMessageQId imu_queueHandle;
osMessageQId debug_queueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* ETH_CODE: add breakpoint when stack oveflow is detected by FreeRTOS.
	* Useful for debugging issues.
	*/
   __BKPT(0);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void startDebugTask(void const * argument);
void StartCtrlSystemTask(void const * argument);

extern void MX_LWIP_Init(void);
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

  /* Create the queue(s) */
  /* definition and creation of imu_queue */
  osMessageQDef(imu_queue, 19, uint8_t);
  imu_queueHandle = osMessageCreate(osMessageQ(imu_queue), NULL);

  /* definition and creation of debug_queue */
  osMessageQDef(debug_queue, 4, control_vars_t);
  debug_queueHandle = osMessageCreate(osMessageQ(debug_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of debugTask */
  osThreadDef(debugTask, startDebugTask, osPriorityIdle, 0, 2048);
  debugTaskHandle = osThreadCreate(osThread(debugTask), NULL);

  /* definition and creation of ctrlSystemTask */
  osThreadDef(ctrlSystemTask, StartCtrlSystemTask, osPriorityIdle, 0, 4096);
  ctrlSystemTaskHandle = osThreadCreate(osThread(ctrlSystemTask), NULL);

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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  rtos_handlers_matrix_t rtos_handler;
  uint32_t number_taks_to_wait = 2;
  uint32_t number_semaphores = 1;
  uint32_t number_queues = 1;
  uint32_t hand_size = sizeof(osThreadId);


  rtos_handler.thread_vector.size = number_taks_to_wait;
  rtos_handler.thread_vector.handler = (osThreadId*) malloc(number_taks_to_wait * hand_size);
  *(rtos_handler.thread_vector.handler) = ctrlSystemTaskHandle;
  *(rtos_handler.thread_vector.handler + 1) = debugTaskHandle;


//	cpp_main_process(NULL, 0);

  main_cpp_wrapper(&rtos_handler);
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_startDebugTask */
/**
* @brief Function implementing the debugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDebugTask */
void startDebugTask(void const * argument)
{
  /* USER CODE BEGIN startDebugTask */
	xTaskNotifyWait(0x00, /* Don't clear any notification bits on entry. */
						ULONG_MAX, /* Reset the notification value to 0 on exit. */
						NULL, /* Notification value is not required */
						portMAX_DELAY); /* Block indefinitely. */

	rtos_handlers_matrix_t rtos_handler;
	uint32_t number_taks_to_wait = 2;
	uint32_t number_semaphores = 1;
	uint32_t number_queues = 1;

	uint32_t hand_size = sizeof(osMessageQId);

	rtos_handler.queue_vector.size = number_queues;
	rtos_handler.queue_vector.handler = (osMessageQId*) malloc(number_queues * hand_size);
	*(rtos_handler.queue_vector.handler) = imu_queueHandle;

	hand_size = sizeof(osThreadId);
	rtos_handler.thread_vector.size = number_taks_to_wait;
	rtos_handler.thread_vector.handler = (osThreadId*) malloc(number_taks_to_wait * hand_size);
	*(rtos_handler.thread_vector.handler) = defaultTaskHandle;
	*(rtos_handler.thread_vector.handler + 1) = ctrlSystemTaskHandle;



	debug_cpp_wrapper(&rtos_handler);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startDebugTask */
}

/* USER CODE BEGIN Header_StartCtrlSystemTask */
/**
* @brief Function implementing the ctrlSystemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCtrlSystemTask */
void StartCtrlSystemTask(void const * argument)
{
  /* USER CODE BEGIN StartCtrlSystemTask */
	xTaskNotifyWait(0x00, /* Don't clear any notification bits on entry. */
						ULONG_MAX, /* Reset the notification value to 0 on exit. */
						NULL, /* Notification value is not required */
						portMAX_DELAY); /* Block indefinitely. */

	rtos_handlers_matrix_t rtos_handler;
	uint32_t number_taks_to_wait = 2;
	uint32_t number_semaphores = 1;
	uint32_t number_queues = 1;

	uint32_t hand_size = sizeof(osMessageQId);

	rtos_handler.queue_vector.size = number_queues;
	rtos_handler.queue_vector.handler = (osMessageQId*) malloc(number_queues * hand_size);
	*(rtos_handler.queue_vector.handler) = imu_queueHandle;

	hand_size = sizeof(osThreadId);
	rtos_handler.thread_vector.size = number_taks_to_wait;
	rtos_handler.thread_vector.handler = (osThreadId*) malloc(number_taks_to_wait * hand_size);
	*(rtos_handler.thread_vector.handler) = defaultTaskHandle;
	*(rtos_handler.thread_vector.handler + 1) = debugTaskHandle;

	seat_ctrl_cpp_wrapper(&rtos_handler);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCtrlSystemTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
