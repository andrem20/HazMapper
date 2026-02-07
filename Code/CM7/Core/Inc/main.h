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
#include "stm32h7xx_hal.h"

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
#define Front_In1_M1_Pin GPIO_PIN_2
#define Front_In1_M1_GPIO_Port GPIOE
#define UltrassonicSensors_Pin GPIO_PIN_3
#define UltrassonicSensors_GPIO_Port GPIOE
#define W_R_GAS_Pin GPIO_PIN_6
#define W_R_GAS_GPIO_Port GPIOF
#define INT_GAS_Pin GPIO_PIN_10
#define INT_GAS_GPIO_Port GPIOF
#define INT_GAS_EXTI_IRQn EXTI15_10_IRQn
#define PWM4_Pin GPIO_PIN_0
#define PWM4_GPIO_Port GPIOA
#define On_Off_Pin GPIO_PIN_0
#define On_Off_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define DCMI_RSET_Pin GPIO_PIN_8
#define DCMI_RSET_GPIO_Port GPIOE
#define V_1_Pin GPIO_PIN_9
#define V_1_GPIO_Port GPIOE
#define A_1_Pin GPIO_PIN_11
#define A_1_GPIO_Port GPIOE
#define Back_In2_M3_Pin GPIO_PIN_13
#define Back_In2_M3_GPIO_Port GPIOE
#define Back_In3_M4_Pin GPIO_PIN_14
#define Back_In3_M4_GPIO_Port GPIOE
#define Back_In4_M4_Pin GPIO_PIN_15
#define Back_In4_M4_GPIO_Port GPIOE
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOB
#define Front_In2_M1_Pin GPIO_PIN_15
#define Front_In2_M1_GPIO_Port GPIOB
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define Back_In1_M3_Pin GPIO_PIN_11
#define Back_In1_M3_GPIO_Port GPIOD
#define V_3_Pin GPIO_PIN_12
#define V_3_GPIO_Port GPIOD
#define A_3_Pin GPIO_PIN_13
#define A_3_GPIO_Port GPIOD
#define Front_In3_M2_Pin GPIO_PIN_14
#define Front_In3_M2_GPIO_Port GPIOD
#define Front_In4_M2_Pin GPIO_PIN_15
#define Front_In4_M2_GPIO_Port GPIOD
#define V_4_Pin GPIO_PIN_6
#define V_4_GPIO_Port GPIOC
#define A_4_Pin GPIO_PIN_7
#define A_4_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOC
#define DCMI_MCLK_Pin GPIO_PIN_8
#define DCMI_MCLK_GPIO_Port GPIOA
#define V_2_Pin GPIO_PIN_15
#define V_2_GPIO_Port GPIOA
#define A_2_Pin GPIO_PIN_3
#define A_2_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_5
#define PWM2_GPIO_Port GPIOB
#define ORANGE_LED_Pin GPIO_PIN_1
#define ORANGE_LED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
