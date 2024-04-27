/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define relay_Pin GPIO_PIN_10
#define relay_GPIO_Port GPIOC
#define KEYBOARD_X_0_Pin GPIO_PIN_0
#define KEYBOARD_X_0_GPIO_Port GPIOD
#define KEYBOARD_X_1_Pin GPIO_PIN_1
#define KEYBOARD_X_1_GPIO_Port GPIOD
#define KEYBOARD_X_2_Pin GPIO_PIN_2
#define KEYBOARD_X_2_GPIO_Port GPIOD
#define KEYBOARD_X_3_Pin GPIO_PIN_3
#define KEYBOARD_X_3_GPIO_Port GPIOD
#define KEYBOARD_Y_0_Pin GPIO_PIN_4
#define KEYBOARD_Y_0_GPIO_Port GPIOD
#define KEYBOARD_Y_1_Pin GPIO_PIN_5
#define KEYBOARD_Y_1_GPIO_Port GPIOD
#define KEYBOARD_Y_2_Pin GPIO_PIN_6
#define KEYBOARD_Y_2_GPIO_Port GPIOD
#define KEYBOARD_Y_3_Pin GPIO_PIN_7
#define KEYBOARD_Y_3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
