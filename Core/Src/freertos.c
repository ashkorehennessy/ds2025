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
#include "icm20948.h"
#include "tim.h"
#include "wheel.h"
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
/* Definitions for blink */
osThreadId_t blinkHandle;
uint32_t blinkBuffer[ 128 ];
osStaticThreadDef_t blinkControlBlock;
const osThreadAttr_t blink_attributes = {
  .name = "blink",
  .cb_mem = &blinkControlBlock,
  .cb_size = sizeof(blinkControlBlock),
  .stack_mem = &blinkBuffer[0],
  .stack_size = sizeof(blinkBuffer),
  .priority = (osPriority_t) osPriorityLow,
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
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for icm20948 */
osThreadId_t icm20948Handle;
uint32_t icm20948Buffer[ 256 ];
osStaticThreadDef_t icm20948ControlBlock;
const osThreadAttr_t icm20948_attributes = {
  .name = "icm20948",
  .cb_mem = &icm20948ControlBlock,
  .cb_size = sizeof(icm20948ControlBlock),
  .stack_mem = &icm20948Buffer[0],
  .stack_size = sizeof(icm20948Buffer),
  .priority = (osPriority_t) osPriorityHigh3,
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
void blinkTask(void *argument);
void angleTask(void *argument);
void UITask(void *argument);
void icm20948Task(void *argument);
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

  /* creation of blink */
  blinkHandle = osThreadNew(blinkTask, NULL, &blink_attributes);

  /* creation of angle */
  angleHandle = osThreadNew(angleTask, NULL, &angle_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(UITask, NULL, &UI_attributes);

  /* creation of icm20948 */
  icm20948Handle = osThreadNew(icm20948Task, NULL, &icm20948_attributes);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_blinkTask */
/**
* @brief Function implementing the blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blinkTask */
void blinkTask(void *argument)
{
  /* USER CODE BEGIN blinkTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
  /* USER CODE END blinkTask */
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
  /* Infinite loop */
  uint32_t adc_val = 0;
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    angle_adc = adc_val * 360.0f / 4095.0f;
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

/* USER CODE BEGIN Header_icm20948Task */
/**
* @brief Function implementing the icm20948 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_icm20948Task */
void icm20948Task(void *argument)
{
  /* USER CODE BEGIN icm20948Task */
  uint16_t last_count = 0;
  HAL_TIM_Base_Start(&htim4);
  // icm20948_init();
  // ak09916_init();
  /* Infinite loop */
  for(;;)
  {
    // icm20948_gyro_read_dps(&gyro);
    // ak09916_mag_read(&mag);
    // angle yaw
    const uint16_t now = __HAL_TIM_GET_COUNTER(&htim4);
    const uint16_t delta = (now >= last_count) ? (now - last_count) : (0xFFFF - last_count + now + 1);
    last_count = now;
    const float dt = (float)delta * 1e-6f;
    angle_yaw += gyro.z * dt;
    // angle azimuth

    osDelay(1);
  }
  /* USER CODE END icm20948Task */
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
  Whell motor;
  whell_init(&motor, &htim3, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
    speed = whell_get_speed(&motor);
    whell_set_speed(&motor, speed_setpoint);
    osDelay(10);
  }
  /* USER CODE END controlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

