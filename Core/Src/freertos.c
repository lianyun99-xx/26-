/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "motor6014.h"
#include "usart.h"
#include "3508_driver.h"
#include "Attitude_solution.h"
#include "fdcan.h"
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
/* Definitions for Motor6014Task */
osThreadId_t Motor6014TaskHandle;
const osThreadAttr_t Motor6014Task_attributes = {
  .name = "Motor6014Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for BMI088 */
osThreadId_t BMI088Handle;
const osThreadAttr_t BMI088_attributes = {
  .name = "BMI088",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AttitudeControl */
osThreadId_t AttitudeControlHandle;
const osThreadAttr_t AttitudeControl_attributes = {
  .name = "AttitudeControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Moror3508 */
osThreadId_t Moror3508Handle;
const osThreadAttr_t Moror3508_attributes = {
  .name = "Moror3508",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void D3508_ApplicationControl(float dt);

/* USER CODE END FunctionPrototypes */

void StartMotor6014Task(void *argument);
void StartBMI088Task(void *argument);
void StartAttitudeControlTask(void *argument);
void StartMotor3508Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Motor6014Task */
  Motor6014TaskHandle = osThreadNew(StartMotor6014Task, NULL, &Motor6014Task_attributes);

  /* creation of BMI088 */
  BMI088Handle = osThreadNew(StartBMI088Task, NULL, &BMI088_attributes);

  /* creation of AttitudeControl */
  AttitudeControlHandle = osThreadNew(StartAttitudeControlTask, NULL, &AttitudeControl_attributes);

  /* creation of Moror3508 */
  Moror3508Handle = osThreadNew(StartMotor3508Task, NULL, &Moror3508_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (!Motor6014TaskHandle || !BMI088Handle || !AttitudeControlHandle || !Moror3508Handle)
    Error_Handler();
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotor6014Task */
/**
  * @brief  Function implementing the Motror6014Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotor6014Task */
void StartMotor6014Task(void *argument)
{
  /* USER CODE BEGIN StartMotor6014Task */
  /* Infinite loop */
  for(;;)
  {
	  Motor6014_SetCommand(&huart3, 1, MOTOR6014_MODE_FOC,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
	  Motor6014_SendCommand(&huart3, 1);
	  Motor6014_SetCommand(&huart3, 2, MOTOR6014_MODE_FOC,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
	  Motor6014_SendCommand(&huart3, 2);
	  Motor6014_SetCommand(&huart3, 3, MOTOR6014_MODE_FOC,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
	  Motor6014_SendCommand(&huart3, 3);
  }
  /* USER CODE END StartMotor6014Task */
}

/* USER CODE BEGIN Header_StartBMI088Task */
/**
* @brief Function implementing the BMI088 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBMI088Task */
void StartBMI088Task(void *argument)
{
  /* USER CODE BEGIN StartBMI088Task */
  TickType_t wake = xTaskGetTickCount();
  for(;;)
  {
    (void)BMI088Read(); /* Sample + calibration + attitude; no initialization. */
    (void)osThreadFlagsSet(AttitudeControlHandle, 1U);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(BMI088_SAMPLE_PERIOD_MS));
  }
  /* USER CODE END StartBMI088Task */
}

/* USER CODE BEGIN Header_StartAttitudeControlTask */
/**
* @brief Function implementing the AttitudeControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAttitudeControlTask */
void StartAttitudeControlTask(void *argument)
{
  /* USER CODE BEGIN StartAttitudeControlTask */
  for(;;)
  {
    (void)osThreadFlagsWait(1U, osFlagsWaitAny, pdMS_TO_TICKS(5));
    /* Attitude is already updated by BMI088Read. Application control can read
     * an Attitude_GetSnapshot here; do not access SPI or update it twice. */
  }
  /* USER CODE END StartAttitudeControlTask */
}

/* USER CODE BEGIN Header_StartMotor3508Task */
/**
* @brief Function implementing the Moror3508 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotor3508Task */
void StartMotor3508Task(void *argument)
{
  /* USER CODE BEGIN StartMotor3508Task */
  TickType_t wake = xTaskGetTickCount();
  TickType_t previous = wake;
  for(;;)
  {
    TickType_t now = xTaskGetTickCount();
    float dt = now == previous ? D3508_DEFAULT_DT : (float)(now - previous) / configTICK_RATE_HZ;
    previous = now;
    D3508_Maintain();
    D3508_ApplicationControl(dt); /* Call Motor3508_SetCommand for each motor. */
    Motor3508_SendCommand(&hfdcan1); /* One 0x200 group per bus after all Sets. */
    Motor3508_SendCommand(&hfdcan2);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
  }
  /* USER CODE END StartMotor3508Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Override to call Motor3508_SetCommand(handle, C620_ID, ticks, rpm, wheel_nm,
 * kp, kd). It computes current immediately; the task then sends each bus once.
 * Startup auto-enable is armed by D3508_Init; no Set means zero current. */
__weak void D3508_ApplicationControl(float dt) { (void)dt; }

/* USER CODE END Application */

