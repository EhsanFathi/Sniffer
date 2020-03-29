/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOF
#define LCD_RW_Pin GPIO_PIN_1
#define LCD_RW_GPIO_Port GPIOF
#define LCD_E_Pin GPIO_PIN_2
#define LCD_E_GPIO_Port GPIOF
#define LCD_D7_Pin GPIO_PIN_4
#define LCD_D7_GPIO_Port GPIOF
#define LCD_D6_Pin GPIO_PIN_5
#define LCD_D6_GPIO_Port GPIOF
#define LCD_D5_Pin GPIO_PIN_6
#define LCD_D5_GPIO_Port GPIOF
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOF
#define key1_Pin GPIO_PIN_8
#define key1_GPIO_Port GPIOF
#define key2_Pin GPIO_PIN_9
#define key2_GPIO_Port GPIOF
#define key3_Pin GPIO_PIN_10
#define key3_GPIO_Port GPIOF
#define key4_Pin GPIO_PIN_11
#define key4_GPIO_Port GPIOF
#define key5_Pin GPIO_PIN_12
#define key5_GPIO_Port GPIOF
#define key6_Pin GPIO_PIN_13
#define key6_GPIO_Port GPIOF
#define key7_Pin GPIO_PIN_14
#define key7_GPIO_Port GPIOF
#define key8_Pin GPIO_PIN_15
#define key8_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
