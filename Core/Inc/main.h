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
#define ADC1_UI_O_Pin GPIO_PIN_0
#define ADC1_UI_O_GPIO_Port GPIOC
#define ADC1_VI_O_Pin GPIO_PIN_1
#define ADC1_VI_O_GPIO_Port GPIOC
#define ADC1_WI_O_Pin GPIO_PIN_2
#define ADC1_WI_O_GPIO_Port GPIOC
#define ADC3_POWER_SUPPLY_Pin GPIO_PIN_3
#define ADC3_POWER_SUPPLY_GPIO_Port GPIOC
#define ADC2_SENS_A_Pin GPIO_PIN_0
#define ADC2_SENS_A_GPIO_Port GPIOA
#define ADC2_SENS_B_Pin GPIO_PIN_1
#define ADC2_SENS_B_GPIO_Port GPIOA
#define ADC2_SENS_C_Pin GPIO_PIN_2
#define ADC2_SENS_C_GPIO_Port GPIOA
#define ADC3_TEMP_Pin GPIO_PIN_3
#define ADC3_TEMP_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
#define TIM1_INL_A_Pin GPIO_PIN_13
#define TIM1_INL_A_GPIO_Port GPIOB
#define TIM1_INL_B_Pin GPIO_PIN_14
#define TIM1_INL_B_GPIO_Port GPIOB
#define TIM1_INL_C_Pin GPIO_PIN_15
#define TIM1_INL_C_GPIO_Port GPIOB
#define TIM1_INH_A_Pin GPIO_PIN_8
#define TIM1_INH_A_GPIO_Port GPIOA
#define TIM1_INH_B_Pin GPIO_PIN_9
#define TIM1_INH_B_GPIO_Port GPIOA
#define TIM1_INH_C_Pin GPIO_PIN_10
#define TIM1_INH_C_GPIO_Port GPIOA
#define EN_GATE_Pin GPIO_PIN_5
#define EN_GATE_GPIO_Port GPIOB
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
