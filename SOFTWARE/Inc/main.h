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
#include "stm32f4xx_hal.h"
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
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_1
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_2
#define BIN2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_10
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOB
#define LED_RGB_R_Pin GPIO_PIN_12
#define LED_RGB_R_GPIO_Port GPIOB
#define LED_RGB_G_Pin GPIO_PIN_13
#define LED_RGB_G_GPIO_Port GPIOB
#define LED_RGB_B_Pin GPIO_PIN_14
#define LED_RGB_B_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_3
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA

#define MOTOR0_TIM htim1
#define MOTOR1_TIM htim3

#define MOTOR_PWM_TIM htim2
#define MOTOR0_PWM_TIM_CHANNEL TIM_CHANNEL_3
#define MOTOR1_PWM_TIM_CHANNEL TIM_CHANNEL_4

#define BUZZER_TIM htim5
#define BUZZER_TIM_CHANNEL TIM_CHANNEL_4

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
