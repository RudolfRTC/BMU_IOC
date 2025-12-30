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
#define ISOSPI_EN_Pin GPIO_PIN_3
#define ISOSPI_EN_GPIO_Port GPIOE
#define TMP_ALRT_Pin GPIO_PIN_13
#define TMP_ALRT_GPIO_Port GPIOC
#define IN_8_Pin GPIO_PIN_2
#define IN_8_GPIO_Port GPIOF
#define IN_7_Pin GPIO_PIN_3
#define IN_7_GPIO_Port GPIOF
#define IN_6_Pin GPIO_PIN_4
#define IN_6_GPIO_Port GPIOF
#define IN_5_Pin GPIO_PIN_5
#define IN_5_GPIO_Port GPIOF
#define IN_4_Pin GPIO_PIN_6
#define IN_4_GPIO_Port GPIOF
#define IN_3_Pin GPIO_PIN_7
#define IN_3_GPIO_Port GPIOF
#define IN_2_Pin GPIO_PIN_8
#define IN_2_GPIO_Port GPIOF
#define IN_1_Pin GPIO_PIN_9
#define IN_1_GPIO_Port GPIOF
#define IN_0_Pin GPIO_PIN_10
#define IN_0_GPIO_Port GPIOF
#define PWR_FLT_Pin GPIO_PIN_2
#define PWR_FLT_GPIO_Port GPIOB
#define PG_5V_Pin GPIO_PIN_11
#define PG_5V_GPIO_Port GPIOF
#define PG_3V3A_Pin GPIO_PIN_12
#define PG_3V3A_GPIO_Port GPIOF
#define PWR_EN_Pin GPIO_PIN_13
#define PWR_EN_GPIO_Port GPIOF
#define PWR_SLEEP_Pin GPIO_PIN_14
#define PWR_SLEEP_GPIO_Port GPIOF
#define CAN_RS_Pin GPIO_PIN_15
#define CAN_RS_GPIO_Port GPIOF
#define EN_5V_Pin GPIO_PIN_7
#define EN_5V_GPIO_Port GPIOE
#define EN_3V3A_Pin GPIO_PIN_8
#define EN_3V3A_GPIO_Port GPIOE
#define OC_7_Pin GPIO_PIN_9
#define OC_7_GPIO_Port GPIOE
#define DEN_0_Pin GPIO_PIN_10
#define DEN_0_GPIO_Port GPIOE
#define DSEL0_0_Pin GPIO_PIN_11
#define DSEL0_0_GPIO_Port GPIOE
#define DSEL1_0_Pin GPIO_PIN_12
#define DSEL1_0_GPIO_Port GPIOE
#define OUT0_0_Pin GPIO_PIN_13
#define OUT0_0_GPIO_Port GPIOE
#define OUT1_0_Pin GPIO_PIN_14
#define OUT1_0_GPIO_Port GPIOE
#define OUT2_0_Pin GPIO_PIN_15
#define OUT2_0_GPIO_Port GPIOE
#define OUT3_0_Pin GPIO_PIN_10
#define OUT3_0_GPIO_Port GPIOB
#define OC_2_Pin GPIO_PIN_11
#define OC_2_GPIO_Port GPIOB
#define CAN2_RS_Pin GPIO_PIN_14
#define CAN2_RS_GPIO_Port GPIOB
#define DEN_1_Pin GPIO_PIN_15
#define DEN_1_GPIO_Port GPIOB
#define DSEL0_1_Pin GPIO_PIN_8
#define DSEL0_1_GPIO_Port GPIOD
#define DSEL1_1_Pin GPIO_PIN_9
#define DSEL1_1_GPIO_Port GPIOD
#define OUT0_1_Pin GPIO_PIN_10
#define OUT0_1_GPIO_Port GPIOD
#define OUT1_1_Pin GPIO_PIN_11
#define OUT1_1_GPIO_Port GPIOD
#define OUT2_1_Pin GPIO_PIN_12
#define OUT2_1_GPIO_Port GPIOD
#define OUT3_1_Pin GPIO_PIN_13
#define OUT3_1_GPIO_Port GPIOD
#define DEN_2_Pin GPIO_PIN_14
#define DEN_2_GPIO_Port GPIOD
#define DSEL0_2_Pin GPIO_PIN_15
#define DSEL0_2_GPIO_Port GPIOD
#define DSEL1_2_Pin GPIO_PIN_2
#define DSEL1_2_GPIO_Port GPIOG
#define OUT0_2_Pin GPIO_PIN_3
#define OUT0_2_GPIO_Port GPIOG
#define OUT1_2_Pin GPIO_PIN_4
#define OUT1_2_GPIO_Port GPIOG
#define OUT2_2_Pin GPIO_PIN_5
#define OUT2_2_GPIO_Port GPIOG
#define OUT3_2_Pin GPIO_PIN_6
#define OUT3_2_GPIO_Port GPIOG
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOG
#define OC_9_Pin GPIO_PIN_8
#define OC_9_GPIO_Port GPIOG
#define OC_4_Pin GPIO_PIN_6
#define OC_4_GPIO_Port GPIOC
#define DEN_3_Pin GPIO_PIN_7
#define DEN_3_GPIO_Port GPIOC
#define DSEL0_3_Pin GPIO_PIN_8
#define DSEL0_3_GPIO_Port GPIOC
#define DSEL1_3_Pin GPIO_PIN_9
#define DSEL1_3_GPIO_Port GPIOC
#define OUT0_3_Pin GPIO_PIN_8
#define OUT0_3_GPIO_Port GPIOA
#define OUT1_3_Pin GPIO_PIN_9
#define OUT1_3_GPIO_Port GPIOA
#define OUT2_3_Pin GPIO_PIN_10
#define OUT2_3_GPIO_Port GPIOA
#define OUT3_3_Pin GPIO_PIN_11
#define OUT3_3_GPIO_Port GPIOA
#define WP_FRAM_Pin GPIO_PIN_12
#define WP_FRAM_GPIO_Port GPIOA
#define DEN_4_Pin GPIO_PIN_15
#define DEN_4_GPIO_Port GPIOA
#define DSEL0_4_Pin GPIO_PIN_10
#define DSEL0_4_GPIO_Port GPIOC
#define DSEL1_4_Pin GPIO_PIN_11
#define DSEL1_4_GPIO_Port GPIOC
#define OUT0_4_Pin GPIO_PIN_12
#define OUT0_4_GPIO_Port GPIOC
#define OUT1_4_Pin GPIO_PIN_0
#define OUT1_4_GPIO_Port GPIOD
#define OUT2_4_Pin GPIO_PIN_1
#define OUT2_4_GPIO_Port GPIOD
#define OUT3_4_Pin GPIO_PIN_2
#define OUT3_4_GPIO_Port GPIOD
#define DEN_5_Pin GPIO_PIN_3
#define DEN_5_GPIO_Port GPIOD
#define DSEL0_5_Pin GPIO_PIN_4
#define DSEL0_5_GPIO_Port GPIOD
#define DSEL1_5_Pin GPIO_PIN_5
#define DSEL1_5_GPIO_Port GPIOD
#define OUT0_5_Pin GPIO_PIN_6
#define OUT0_5_GPIO_Port GPIOD
#define OUT1_5_Pin GPIO_PIN_7
#define OUT1_5_GPIO_Port GPIOD
#define OUT2_5_Pin GPIO_PIN_9
#define OUT2_5_GPIO_Port GPIOG
#define OUT3_5_Pin GPIO_PIN_10
#define OUT3_5_GPIO_Port GPIOG
#define IN_19_Pin GPIO_PIN_11
#define IN_19_GPIO_Port GPIOG
#define IN_18_Pin GPIO_PIN_12
#define IN_18_GPIO_Port GPIOG
#define IN_17_Pin GPIO_PIN_13
#define IN_17_GPIO_Port GPIOG
#define IN_16_Pin GPIO_PIN_14
#define IN_16_GPIO_Port GPIOG
#define IN_15_Pin GPIO_PIN_15
#define IN_15_GPIO_Port GPIOG
#define IN_14_Pin GPIO_PIN_4
#define IN_14_GPIO_Port GPIOB
#define IN_13_Pin GPIO_PIN_5
#define IN_13_GPIO_Port GPIOB
#define IN_12_Pin GPIO_PIN_8
#define IN_12_GPIO_Port GPIOB
#define IN_11_Pin GPIO_PIN_9
#define IN_11_GPIO_Port GPIOB
#define IN_10_Pin GPIO_PIN_0
#define IN_10_GPIO_Port GPIOE
#define IN_9_Pin GPIO_PIN_1
#define IN_9_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
