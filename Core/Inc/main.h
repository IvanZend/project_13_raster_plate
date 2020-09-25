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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_file_13.h"
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
#define ON_TOMO_Pin GPIO_PIN_1
#define ON_TOMO_GPIO_Port GPIOF
#define BUCKY_BRAKE_Pin GPIO_PIN_10
#define BUCKY_BRAKE_GPIO_Port GPIOG
#define CONFIG_3_Pin GPIO_PIN_0
#define CONFIG_3_GPIO_Port GPIOA
#define CONFIG_2_Pin GPIO_PIN_1
#define CONFIG_2_GPIO_Port GPIOA
#define CONFIG_1_Pin GPIO_PIN_2
#define CONFIG_1_GPIO_Port GPIOA
#define GRID_120_Pin GPIO_PIN_3
#define GRID_120_GPIO_Port GPIOA
#define GRID_180_Pin GPIO_PIN_4
#define GRID_180_GPIO_Port GPIOA
#define BUCKY_READY_Pin GPIO_PIN_5
#define BUCKY_READY_GPIO_Port GPIOA
#define BUCKY_CALL_Pin GPIO_PIN_6
#define BUCKY_CALL_GPIO_Port GPIOA
#define GRID_BUTTON_Pin GPIO_PIN_7
#define GRID_BUTTON_GPIO_Port GPIOA
#define LASER_CENTERING_Pin GPIO_PIN_0
#define LASER_CENTERING_GPIO_Port GPIOB
#define GRID_END_POINT_Pin GPIO_PIN_8
#define GRID_END_POINT_GPIO_Port GPIOA
#define GRID_120_DETECT_Pin GPIO_PIN_9
#define GRID_120_DETECT_GPIO_Port GPIOA
#define GRID_180_DETECT_Pin GPIO_PIN_10
#define GRID_180_DETECT_GPIO_Port GPIOA
#define PUSHBUTTON_BUCKYBRAKE_Pin GPIO_PIN_15
#define PUSHBUTTON_BUCKYBRAKE_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_4
#define RESET_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_5
#define ENABLE_GPIO_Port GPIOB
#define CURRENT_WIND_Pin GPIO_PIN_6
#define CURRENT_WIND_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_7
#define STEP_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_8
#define DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
