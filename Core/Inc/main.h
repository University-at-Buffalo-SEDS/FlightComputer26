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
#include "stm32h5xx_hal.h"

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
#define CS_BARO_Pin GPIO_PIN_3
#define CS_BARO_GPIO_Port GPIOC
#define CS_ACCEL_Pin GPIO_PIN_2
#define CS_ACCEL_GPIO_Port GPIOA
#define CS_GYRO_Pin GPIO_PIN_3
#define CS_GYRO_GPIO_Port GPIOA
#define ACCL_EXTI_1_Pin GPIO_PIN_4
#define ACCL_EXTI_1_GPIO_Port GPIOC
#define ACCL_EXTI_1_EXTI_IRQn EXTI4_IRQn
#define ACCL_EXTI_2_Pin GPIO_PIN_5
#define ACCL_EXTI_2_GPIO_Port GPIOC
#define ACCL_EXTI_2_EXTI_IRQn EXTI5_IRQn
#define GYRO_EXTI_1_Pin GPIO_PIN_0
#define GYRO_EXTI_1_GPIO_Port GPIOB
#define GYRO_EXTI_1_EXTI_IRQn EXTI0_IRQn
#define GYRO_EXTI_2_Pin GPIO_PIN_1
#define GYRO_EXTI_2_GPIO_Port GPIOB
#define GYRO_EXTI_2_EXTI_IRQn EXTI1_IRQn
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define CONFIGURABLE_Pin GPIO_PIN_14
#define CONFIGURABLE_GPIO_Port GPIOB
#define BARO_EXTI_Pin GPIO_PIN_7
#define BARO_EXTI_GPIO_Port GPIOC
#define BARO_EXTI_EXTI_IRQn EXTI7_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
