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
#define TFT_RST_Pin GPIO_PIN_2
#define TFT_RST_GPIO_Port GPIOE
#define ENCODER_LEFT_A_Pin GPIO_PIN_0
#define ENCODER_LEFT_A_GPIO_Port GPIOA
#define ENCODER_LEFT_B_Pin GPIO_PIN_1
#define ENCODER_LEFT_B_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOA
#define ENCODER_RIGHT_A_Pin GPIO_PIN_6
#define ENCODER_RIGHT_A_GPIO_Port GPIOA
#define ENCODER_RIGHT_B_Pin GPIO_PIN_7
#define ENCODER_RIGHT_B_GPIO_Port GPIOA
#define GRAY_AD1_Pin GPIO_PIN_5
#define GRAY_AD1_GPIO_Port GPIOC
#define TFT_DC_Pin GPIO_PIN_0
#define TFT_DC_GPIO_Port GPIOB
#define GRAY_AD0_Pin GPIO_PIN_7
#define GRAY_AD0_GPIO_Port GPIOE
#define GRAY_OUT_Pin GPIO_PIN_9
#define GRAY_OUT_GPIO_Port GPIOE
#define HCSR04_ECHO_Pin GPIO_PIN_12
#define HCSR04_ECHO_GPIO_Port GPIOB
#define HCSR04_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define HCSR04_TRIG_Pin GPIO_PIN_14
#define HCSR04_TRIG_GPIO_Port GPIOB
#define MOTOR_LEFT_IN1_Pin GPIO_PIN_12
#define MOTOR_LEFT_IN1_GPIO_Port GPIOD
#define MOTOR_LEFT_IN2_Pin GPIO_PIN_13
#define MOTOR_LEFT_IN2_GPIO_Port GPIOD
#define MOTOR_RIGHT_IN1_Pin GPIO_PIN_14
#define MOTOR_RIGHT_IN1_GPIO_Port GPIOD
#define MOTOR_RIGHT_IN2_Pin GPIO_PIN_15
#define MOTOR_RIGHT_IN2_GPIO_Port GPIOD
#define GRAY_AD2_Pin GPIO_PIN_9
#define GRAY_AD2_GPIO_Port GPIOC
#define RADAR_RIGHT_OT2_Pin GPIO_PIN_8
#define RADAR_RIGHT_OT2_GPIO_Port GPIOA
#define RADAR_LEFT_OT2_Pin GPIO_PIN_9
#define RADAR_LEFT_OT2_GPIO_Port GPIOA
#define BNO_RST_Pin GPIO_PIN_4
#define BNO_RST_GPIO_Port GPIOB
#define BNO_INT_Pin GPIO_PIN_5
#define BNO_INT_GPIO_Port GPIOB
#define BNO_SCL_Pin GPIO_PIN_6
#define BNO_SCL_GPIO_Port GPIOB
#define BNO_SDA_Pin GPIO_PIN_7
#define BNO_SDA_GPIO_Port GPIOB
#define TFT_BL_Pin GPIO_PIN_8
#define TFT_BL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
