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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef enum
{
	START,
	SETUP,
	DISCHARGE,
	PAUSED,
	FINISH
} FSM_states;

/*
 * 	SETUP_PARAM_DISCHARGE_CURRENT,
	SETUP_PARAM_CUTOFF_VOLTAGE,
	SETUP_PARAM_COUNT//calculate max enum value for incrementing in callback modes
 */
typedef enum
{
	SETUP_PARAM_DISCHARGE_CURRENT,
	SETUP_PARAM_CUTOFF_VOLTAGE,

	SETUP_PARAM_COUNT
} SETUP_set;

/*
 * @brief Parameters for discharge FSM value display
 */
typedef struct {
    float voltage;
    float current_ma;
    float capacity_mah;
    uint32_t elapsed_time_sec;
} DischargeDisplayData_t;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define CHARGING_STATE_Pin GPIO_PIN_14
#define CHARGING_STATE_GPIO_Port GPIOC
#define Button_mode_Pin GPIO_PIN_1
#define Button_mode_GPIO_Port GPIOB
#define Button_mode_EXTI_IRQn EXTI1_IRQn
#define Button_add_Pin GPIO_PIN_10
#define Button_add_GPIO_Port GPIOB
#define Button_add_EXTI_IRQn EXTI15_10_IRQn
#define Button_sub_Pin GPIO_PIN_11
#define Button_sub_GPIO_Port GPIOB
#define Button_sub_EXTI_IRQn EXTI15_10_IRQn
#define RS_Pin GPIO_PIN_9
#define RS_GPIO_Port GPIOA
#define RW_Pin GPIO_PIN_10
#define RW_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_5
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_6
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_7
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_8
#define D7_GPIO_Port GPIOB
#define PWM_MOSFET_Pin GPIO_PIN_9
#define PWM_MOSFET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
