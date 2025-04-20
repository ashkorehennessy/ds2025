/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "pool.h"
#include "st7735.h"
#include "UI.h"
#include "tim.h"
#include "wheel.h"
#include "tgmath.h"
#include "PID.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for angle */
osThreadId_t angleHandle;
uint32_t angleBuffer[ 128 ];
osStaticThreadDef_t angleControlBlock;
const osThreadAttr_t angle_attributes = {
  .name = "angle",
  .cb_mem = &angleControlBlock,
  .cb_size = sizeof(angleControlBlock),
  .stack_mem = &angleBuffer[0],
  .stack_size = sizeof(angleBuffer),
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for UI */
osThreadId_t UIHandle;
uint32_t UIBuffer[ 512 ];
osStaticThreadDef_t UIControlBlock;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .cb_mem = &UIControlBlock,
  .cb_size = sizeof(UIControlBlock),
  .stack_mem = &UIBuffer[0],
  .stack_size = sizeof(UIBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for control */
osThreadId_t controlHandle;
uint32_t controlBuffer[ 256 ];
osStaticThreadDef_t controlControlBlock;
const osThreadAttr_t control_attributes = {
  .name = "control",
  .cb_mem = &controlControlBlock,
  .cb_size = sizeof(controlControlBlock),
  .stack_mem = &controlBuffer[0],
  .stack_size = sizeof(controlBuffer),
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for task4 */
osThreadId_t task4Handle;
const osThreadAttr_t task4_attributes = {
  .name = "task4",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void angleTask(void *argument);
void UITask(void *argument);
void controlTask(void *argument);
void Task1(void *argument);
void Task4(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of angle */
  angleHandle = osThreadNew(angleTask, NULL, &angle_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(UITask, NULL, &UI_attributes);

  /* creation of control */
  controlHandle = osThreadNew(controlTask, NULL, &control_attributes);

  /* creation of task1 */
  // task1Handle = osThreadNew(Task1, NULL, &task1_attributes);

  /* creation of task4 */
  task4Handle = osThreadNew(Task4, NULL, &task4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_angleTask */
/**
* @brief Function implementing the angle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_angleTask */
void angleTask(void *argument)
{
  /* USER CODE BEGIN angleTask */
  float angle = 0;
  uint32_t adc_sum = 0;
  uint32_t adc_windows[20] = {0};
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    // 更新窗口
    for (int i = 0; i < 19; i++) {
      adc_windows[i] = adc_windows[i + 1];
    }
    adc_windows[19] = adc_raw;
    adc_sum -= adc_windows[0];
    adc_sum += adc_windows[19];
    adc_use = (float)adc_sum / 20;
    angle = adc_use * 360.0f / 4095.0f;  // adc到角度转换
    angle = (angle - 6) * 45 / 44;  // 角度修正
    if (angle < 0) angle += 360.0f;
    angle_adc = angle;
    if (angle_adc > 0) {
      angle_top = -(angle_adc - 180);
    } else {
      angle_top = -(angle_adc + 180);
    }
    osDelay(1);
  }
  /* USER CODE END angleTask */
}

/* USER CODE BEGIN Header_UITask */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UITask */
void UITask(void *argument)
{
  /* USER CODE BEGIN UITask */
  HAL_GPIO_WritePin(bl_GPIO_Port, bl_Pin, GPIO_PIN_SET);
  ST7735_Init();
  UI_init();
  /* Infinite loop */
  for(;;)
  {
    UI_show();
    UI_key_process();
    osDelay(20);
  }
  /* USER CODE END UITask */
}

/* USER CODE BEGIN Header_controlTask */
/**
* @brief Function implementing the control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlTask */
void controlTask(void *argument)
{
  /* USER CODE BEGIN controlTask */
  float last_speed = 0;
  uint32_t tick = osKernelGetTickCount();
  whell_init(&motor, &htim3, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  whell_set_speed(&motor, 0);
  /* Infinite loop */
  for(;;)
  {
    speed = (float)whell_get_speed(&motor) * 0.5f + last_speed * 0.5f;
    last_speed = speed;
    angle_yaw += speed * 0.667f;
    tick += 10;
    osDelayUntil(tick);
  }
  /* USER CODE END controlTask */
}

/* USER CODE BEGIN Header_Task1 */
/**
* @brief Function implementing the task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task1 */
void Task1(void *argument)
{
  /* USER CODE BEGIN Task1 */
  task_running = 1;
  PID_Base yawpid = PID_Base_Init(100,0,1500,7000,-7000,1,0.5f);
  uint32_t tick = osKernelGetTickCount();
  uint32_t invalid_tick = tick;
  /* Infinite loop */
  while (tick - invalid_tick < 400)
  {
    if (fabs(target_angle_yaw - angle_yaw) > 2) {
      invalid_tick = tick;
    }
    yaw_pidout = PID_Base_Calc(&yawpid, angle_yaw, target_angle_yaw);
    if (yaw_pidout > 0)yaw_pidout += deadzone;
    if (yaw_pidout < 0)yaw_pidout -= deadzone;
    whell_set_speed(&motor, (int)yaw_pidout);
    tick += 10;
    osDelayUntil(tick);
  }
  whell_set_speed(&motor, 0);
  task_running = 0;
  osThreadExit();
  /* USER CODE END Task1 */
}

/* USER CODE BEGIN Header_Task4 */
/**
* @brief Function implementing the task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task4 */
void Task4(void *argument)
{
  /* USER CODE BEGIN Task4 */
  task_running = 1;
  deadzone = 600;
  PID_Base yawpid = PID_Base_Init(-500,-0,-200,5000,-5000,0,0.5);
  PID_Base adcpid = PID_Base_Init(-0.03f,-0.02f,-0.15f,50,-50,0,0.5);
  uint32_t tick = osKernelGetTickCount();
  uint32_t invalid_tick = tick;
  float last_angle_yaw = 0;
  /* Infinite loop */
  while (tick - invalid_tick < 50000)
  {
    float angle_setpoint;
    if (fabs(angle_top) > 30) {
      invalid_tick = tick;
    }
    if(fabs(angle_top) < 3) {
      angle_setpoint = log(fabs(angle_yaw));
      if (angle_setpoint < 0) angle_setpoint = 0;
      if (angle_yaw < 0) angle_setpoint = -angle_setpoint;
    } else {
      angle_setpoint = 0;
    }
    float diff = angle_yaw - last_angle_yaw;
    last_angle_yaw = angle_yaw;
    adc_pidout = PID_Base_Calc(&adcpid, adc_use, 2060 - diff * 10 - angle_yaw);
    yaw_pidout = PID_Base_Calc(&yawpid, speed  ,-adc_pidout);
    if (yaw_pidout > 0)yaw_pidout += deadzone;
    if (yaw_pidout < 0)yaw_pidout -= deadzone;
    if (fabs(angle_top) > 30) {
      whell_set_speed(&motor, 0);
    } else {
      whell_set_speed(&motor, (int)yaw_pidout);
    }
    tick += 10;
    osDelayUntil(tick);
  }
  whell_set_speed(&motor, 0);
  task_running = 0;
  osThreadExit();
  /* USER CODE END Task4 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void start_task(int index)
{
  switch (index) {
  case 1: task1Handle = osThreadNew(Task1, NULL, &task1_attributes);break;
  case 4: task4Handle = osThreadNew(Task4, NULL, &task4_attributes);break;
  }
}
/* USER CODE END Application */

