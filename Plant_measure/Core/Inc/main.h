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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "HD44780.h"
#include "myTasks.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define NUMBER_OF_SENSORS 2			//<< CUSTOMIZE THIS VARIABLE TO YOUR PROJECT
#define MAX_MEAS_NUMBER 16			//<< CUSTOMIZE THIS VARIABLE TO YOUR PROJECT
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_LED_Pin GPIO_PIN_13
#define MCU_LED_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_1
#define LCD_RW_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_2
#define LCD_E_GPIO_Port GPIOA
#define LCD_DATA_4_Pin GPIO_PIN_3
#define LCD_DATA_4_GPIO_Port GPIOA
#define LCD_DATA_5_Pin GPIO_PIN_4
#define LCD_DATA_5_GPIO_Port GPIOA
#define LCD_DATA_6_Pin GPIO_PIN_5
#define LCD_DATA_6_GPIO_Port GPIOA
#define LCD_DATA_7_Pin GPIO_PIN_6
#define LCD_DATA_7_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_1
#define LCD_DC_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define PUMP_0_Pin GPIO_PIN_5
#define PUMP_0_GPIO_Port GPIOB
#define PUMP_1_Pin GPIO_PIN_6
#define PUMP_1_GPIO_Port GPIOB
#define PUMP_2_Pin GPIO_PIN_7
#define PUMP_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
