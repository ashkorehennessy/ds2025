/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define blink_Pin GPIO_PIN_13
#define blink_GPIO_Port GPIOC
#define angle_Pin GPIO_PIN_0
#define angle_GPIO_Port GPIOA
#define res_Pin GPIO_PIN_2
#define res_GPIO_Port GPIOA
#define dc_Pin GPIO_PIN_3
#define dc_GPIO_Port GPIOA
#define cs_Pin GPIO_PIN_4
#define cs_GPIO_Port GPIOA
#define scl_Pin GPIO_PIN_5
#define scl_GPIO_Port GPIOA
#define bl_Pin GPIO_PIN_6
#define bl_GPIO_Port GPIOA
#define sda_Pin GPIO_PIN_7
#define sda_GPIO_Port GPIOA
#define icm20948_sclk_Pin GPIO_PIN_0
#define icm20948_sclk_GPIO_Port GPIOB
#define sw3_Pin GPIO_PIN_12
#define sw3_GPIO_Port GPIOB
#define sw2_Pin GPIO_PIN_13
#define sw2_GPIO_Port GPIOB
#define sw1_Pin GPIO_PIN_14
#define sw1_GPIO_Port GPIOB
#define sw0_Pin GPIO_PIN_15
#define sw0_GPIO_Port GPIOB
#define icm20948_sdi_Pin GPIO_PIN_10
#define icm20948_sdi_GPIO_Port GPIOA
#define icm2098_ncs_Pin GPIO_PIN_11
#define icm2098_ncs_GPIO_Port GPIOA
#define icm20948_sdo_Pin GPIO_PIN_12
#define icm20948_sdo_GPIO_Port GPIOA
#define motor_pwm_a_Pin GPIO_PIN_15
#define motor_pwm_a_GPIO_Port GPIOA
#define motor_pwm_b_Pin GPIO_PIN_3
#define motor_pwm_b_GPIO_Port GPIOB
#define encoder_a_Pin GPIO_PIN_4
#define encoder_a_GPIO_Port GPIOB
#define encoder_b_Pin GPIO_PIN_5
#define encoder_b_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
