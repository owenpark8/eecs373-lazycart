/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISP_RS_Pin GPIO_PIN_5
#define DISP_RS_GPIO_Port GPIOF
#define USONIC_L_ECHO_Pin GPIO_PIN_6
#define USONIC_L_ECHO_GPIO_Port GPIOF
#define PRESSURE_ADC_IN1_Pin GPIO_PIN_0
#define PRESSURE_ADC_IN1_GPIO_Port GPIOC
#define USONIC_R_ECHO_Pin GPIO_PIN_0
#define USONIC_R_ECHO_GPIO_Port GPIOA
#define L_FORWARD_Pin GPIO_PIN_15
#define L_FORWARD_GPIO_Port GPIOF
#define L_BACKWARD_Pin GPIO_PIN_0
#define L_BACKWARD_GPIO_Port GPIOG
#define PWM_RIGHT_SERVO_Pin GPIO_PIN_12
#define PWM_RIGHT_SERVO_GPIO_Port GPIOD
#define PWM_LEFT_MOTOR_Pin GPIO_PIN_14
#define PWM_LEFT_MOTOR_GPIO_Port GPIOD
#define PWM_RIGHT_MOTOR_Pin GPIO_PIN_15
#define PWM_RIGHT_MOTOR_GPIO_Port GPIOD
#define USONIC_TRIG_Pin GPIO_PIN_7
#define USONIC_TRIG_GPIO_Port GPIOC
#define DISP_CMD_Pin GPIO_PIN_7
#define DISP_CMD_GPIO_Port GPIOD
#define R_FORWARD_Pin GPIO_PIN_4
#define R_FORWARD_GPIO_Port GPIOB
#define R_BACKWARD_Pin GPIO_PIN_5
#define R_BACKWARD_GPIO_Port GPIOB
#define DISP_CS_Pin GPIO_PIN_6
#define DISP_CS_GPIO_Port GPIOB
#define PWM_LEFT_SERVO_Pin GPIO_PIN_7
#define PWM_LEFT_SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
