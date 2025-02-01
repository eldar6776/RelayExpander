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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "LuxNET.h"
#include "TinyFrame.h"
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
void RS485_Tick(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOA
#define RELAY5_Pin GPIO_PIN_6
#define RELAY5_GPIO_Port GPIOA
#define RELAY6_Pin GPIO_PIN_7
#define RELAY6_GPIO_Port GPIOA
#define RELAY7_Pin GPIO_PIN_0
#define RELAY7_GPIO_Port GPIOB
#define RELAY8_Pin GPIO_PIN_1
#define RELAY8_GPIO_Port GPIOB
#define BUTTON_SELECT_Pin GPIO_PIN_2
#define BUTTON_SELECT_GPIO_Port GPIOB
#define ADDRESS_0_Pin GPIO_PIN_10
#define ADDRESS_0_GPIO_Port GPIOB
#define ADDRESS_1_Pin GPIO_PIN_11
#define ADDRESS_1_GPIO_Port GPIOB
#define ADDRESS_2_Pin GPIO_PIN_12
#define ADDRESS_2_GPIO_Port GPIOB
#define ADDRESS_3_Pin GPIO_PIN_13
#define ADDRESS_3_GPIO_Port GPIOB
#define ADDRESS_4_Pin GPIO_PIN_14
#define ADDRESS_4_GPIO_Port GPIOB
#define ADDRESS_5_Pin GPIO_PIN_15
#define ADDRESS_5_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_8
#define RELAY1_GPIO_Port GPIOA
#define RELAY2_Pin GPIO_PIN_9
#define RELAY2_GPIO_Port GPIOA
#define RELAY3_Pin GPIO_PIN_10
#define RELAY3_GPIO_Port GPIOA
#define RELAY4_Pin GPIO_PIN_11
#define RELAY4_GPIO_Port GPIOA
#define BUTTON_ON_OFF_Pin GPIO_PIN_3
#define BUTTON_ON_OFF_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_4
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_5
#define LED8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern bool init_tf;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
