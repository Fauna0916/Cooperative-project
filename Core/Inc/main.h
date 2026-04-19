/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define ENCODER_LEFT_A_Pin GPIO_PIN_0
#define ENCODER_LEFT_A_GPIO_Port GPIOA
#define ENCODER_LEFT_B_Pin GPIO_PIN_1
#define ENCODER_LEFT_B_GPIO_Port GPIOA
#define ENCODER_RIGHT_A_Pin GPIO_PIN_6
#define ENCODER_RIGHT_A_GPIO_Port GPIOA
#define ENCODER_RIGHT_B_Pin GPIO_PIN_7
#define ENCODER_RIGHT_B_GPIO_Port GPIOA
#define BNO_INT_Pin GPIO_PIN_14
#define BNO_INT_GPIO_Port GPIOE
#define BNO_INT_EXTI_IRQn EXTI15_10_IRQn
#define BNO_RST_Pin GPIO_PIN_15
#define BNO_RST_GPIO_Port GPIOE
#define BNO_SCL_Pin GPIO_PIN_10
#define BNO_SCL_GPIO_Port GPIOB
#define BNO_SDA_Pin GPIO_PIN_11
#define BNO_SDA_GPIO_Port GPIOB
#define MOTOR_LEFT_IN1_Pin GPIO_PIN_12
#define MOTOR_LEFT_IN1_GPIO_Port GPIOD
#define MOTOR_LEFT_IN2_Pin GPIO_PIN_13
#define MOTOR_LEFT_IN2_GPIO_Port GPIOD
#define MOTOR_RIGHT_IN1_Pin GPIO_PIN_14
#define MOTOR_RIGHT_IN1_GPIO_Port GPIOD
#define MOTOR_RIGHT_IN2_Pin GPIO_PIN_15
#define MOTOR_RIGHT_IN2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
