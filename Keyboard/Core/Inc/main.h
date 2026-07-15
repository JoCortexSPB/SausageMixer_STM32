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
#include "stm32f0xx_hal.h"

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
#define LD0_Pin GPIO_PIN_0
#define LD0_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_1
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_2
#define LD2_GPIO_Port GPIOB
#define LE_Pin GPIO_PIN_10
#define LE_GPIO_Port GPIOB
#define BTN_START_Pin GPIO_PIN_12
#define BTN_START_GPIO_Port GPIOB
#define BTN_STOP_Pin GPIO_PIN_13
#define BTN_STOP_GPIO_Port GPIOB
#define BTN_AIR_Pin GPIO_PIN_14
#define BTN_AIR_GPIO_Port GPIOB
#define BTN_RESET_Pin GPIO_PIN_15
#define BTN_RESET_GPIO_Port GPIOB
#define BTN_LEFT_Pin GPIO_PIN_8
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_RIGHT_Pin GPIO_PIN_9
#define BTN_RIGHT_GPIO_Port GPIOA
#define BTN_UP_Pin GPIO_PIN_10
#define BTN_UP_GPIO_Port GPIOA
#define BTN_DOWN_Pin GPIO_PIN_11
#define BTN_DOWN_GPIO_Port GPIOA
#define LAMP_Pin GPIO_PIN_7
#define LAMP_GPIO_Port GPIOF
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_4
#define LD4_GPIO_Port GPIOB
#define LD5_Pin GPIO_PIN_5
#define LD5_GPIO_Port GPIOB
#define LD6_Pin GPIO_PIN_6
#define LD6_GPIO_Port GPIOB
#define LD7_Pin GPIO_PIN_7
#define LD7_GPIO_Port GPIOB
#define LRS_Pin GPIO_PIN_8
#define LRS_GPIO_Port GPIOB
#define LRW_Pin GPIO_PIN_9
#define LRW_GPIO_Port GPIOB

#define CMD_NONE     0x00

#define CMD_START    0x10
#define CMD_STOP     0x11
#define CMD_AIR      0x12
#define CMD_RESET    0x13

#define CMD_LEFT     0x20
#define CMD_RIGHT    0x21
#define CMD_UP       0x22
#define CMD_DOWN     0x23

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
