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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void angleTask(void *argument);
void UITask(void *argument);
void controlTask(void *argument);

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
  float last_angle = 0;

  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    angle = (float)adc_raw * 360.0f / 4095.0f;
    angle -= angle_offset;
    angle = alpha * last_angle + (1 - alpha) * angle;

    angle_adc = angle - 180;
    if (angle_adc < -170) {
      angle_adc+=4;
    }
    if (angle_adc > 80 && angle_adc < 100) {
      angle_adc-=5;
    }
    if (angle_adc > 0) {
      angle_show = angle_adc - 180;
    } else {
      angle_show = angle_adc + 180;
    }
    last_angle = angle;
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
  PID_Base yawpid = PID_Base_Init(-500,0,0,-200,0,5000,-5000,0,0);
  PID_Base adcpid = PID_Base_Init(-0.03f,0,0,-0.08f,0,1000,-1000,0,0);
  Whell motor;
  float last_speed = 0;
  whell_init(&motor, &htim3, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
    speed = (float)whell_get_speed(&motor) * 0.75f + last_speed * 0.25f;
    last_speed = speed;
    angle_yaw += speed * 0.667f;
    adc_pidout = PID_Base_Calc(&adcpid, (float)adc_raw, 0, 2190-angle_yaw/2);
    yaw_pidout = PID_Base_Calc(&yawpid, speed + adc_pidout ,0 ,0);
    if (yaw_pidout > 0)yaw_pidout += deadzone;
    if (yaw_pidout < 0)yaw_pidout -= deadzone;
    if (fabs(angle_adc) > 30) {
      whell_set_speed(&motor, 0);
    } else {
      whell_set_speed(&motor, (int)yaw_pidout);
    }
    osDelay(10);
  }
  /* USER CODE END controlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

