/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define E1_Pin GPIO_PIN_0
#define E1_GPIO_Port GPIOB
#define E1_EXTI_IRQn EXTI0_IRQn
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOB
#define E3_Pin GPIO_PIN_11
#define E3_GPIO_Port GPIOB
#define E3_EXTI_IRQn EXTI15_10_IRQn
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define E4_Pin GPIO_PIN_9
#define E4_GPIO_Port GPIOA
#define E4_EXTI_IRQn EXTI9_5_IRQn
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOA
#define E2_Pin GPIO_PIN_15
#define E2_GPIO_Port GPIOA
#define E2_EXTI_IRQn EXTI15_10_IRQn
#define P41_Pin GPIO_PIN_6
#define P41_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
