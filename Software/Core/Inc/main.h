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
#include "stm32f1xx_hal.h"

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
#define Dir1_Pin GPIO_PIN_0
#define Dir1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_2
#define PWM2_GPIO_Port GPIOA
#define PWM_M8_Pin GPIO_PIN_3
#define PWM_M8_GPIO_Port GPIOA
#define CEO_Pin GPIO_PIN_4
#define CEO_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_5
#define SCLK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define PWM_M7_Pin GPIO_PIN_6
#define PWM_M7_GPIO_Port GPIOC
#define PWM_M6_Pin GPIO_PIN_7
#define PWM_M6_GPIO_Port GPIOC
#define PWM_M5_Pin GPIO_PIN_8
#define PWM_M5_GPIO_Port GPIOC
#define PWM_M4_Pin GPIO_PIN_9
#define PWM_M4_GPIO_Port GPIOC
#define PWM_M3_Pin GPIO_PIN_8
#define PWM_M3_GPIO_Port GPIOA
#define PWM_M2_Pin GPIO_PIN_9
#define PWM_M2_GPIO_Port GPIOA
#define PWM_M1_Pin GPIO_PIN_10
#define PWM_M1_GPIO_Port GPIOA
#define Dir2_Pin GPIO_PIN_15
#define Dir2_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOC
#define RX_Pin GPIO_PIN_11
#define RX_GPIO_Port GPIOC
#define En__Pin GPIO_PIN_2
#define En__GPIO_Port GPIOD
#define SCLB6_Pin GPIO_PIN_6
#define SCLB6_GPIO_Port GPIOB
#define SDAB7_Pin GPIO_PIN_7
#define SDAB7_GPIO_Port GPIOB
#define Servo_Pin GPIO_PIN_8
#define Servo_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
