/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_16x2_PARALLEL.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_BUFFER_SIZE 32
#define R_load 0.22 //load ohms
#define Vrefint 1.2 //1.2V internal reference voltage
#define ADC_steps 4096

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * 	START,
	SETUP,
	DISCHARGE,
	PAUSED,
	FINISH
 */
volatile FSM_states STATE_MCU = START;
volatile SETUP_set SETUP_CONFIGURATION = SETUP_PARAM_DISCHARGE_CURRENT;
char LCD_buffer[LCD_BUFFER_SIZE];
float Vdda = 3.3; //ref for measurements, calculated later by ADC with internal ref
volatile uint8_t updateScreenRequest = 1;//update at setup on first frame
volatile uint16_t Discharge_current = 500;//mA
volatile uint16_t Cutoff_voltage = 0;//V
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DELAY_US(uint16_t TIME_US);
void updateScreen();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	if(HAL_ADCEx_Calibration_Start(&hadc1) !=HAL_OK)
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}
	if(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK)
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}
	if(HAL_TIM_Base_Start_IT(&htim4)!=HAL_OK)
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}
	HAL_Delay(500);// wait for DC point

	//Read internal reference for VDDA
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, 500);
	uint16_t Vadc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	Vdda = Vadc/ADC_steps*Vrefint;

	/*
	 * @brief Init in 4 bit LCD 16x2
	 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(updateScreenRequest)//only perfrom LCD switch states on gpio change
		{
			updateScreen();
		}

		switch(STATE_MCU)
		{
		case  SETUP:
		{
			break;
		}
		case  DISCHARGE:
		{

			break;
		}
		case  FINISH:
		{

			break;
		}
		case  PAUSED:
		{

			break;
		}
		default:
		{

			break;
		}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void DELAY_US(uint16_t TIME_US)
{
	uint32_t old_timer_value = TIM3->CNT;
	uint32_t target_time = (old_timer_value + TIME_US) % (TIM3->ARR + 1);

	if (target_time < old_timer_value)  // Handle timer overflow
	{
		while (TIM3->CNT >= old_timer_value);  // Wait for overflow
	}

	while (TIM3->CNT < target_time);  // Wait until target time is reached
}
void updateScreen()
{
	static uint8_t place;
	updateScreenRequest = 0;

	switch(STATE_MCU)
	{
	case  START:
	{
		LCD_Init();
		sprintf(LCD_buffer,"Battery analyzer");
		LCD_SEND_STR(LCD_buffer, 0, 0);
		sprintf(LCD_buffer,"BatLinux");
		place = (LCD_COLS-strlen(LCD_buffer))/2;
		LCD_SEND_STR(LCD_buffer, place, 1);
		HAL_Delay(1000);
		STATE_MCU = SETUP;
		break;
	}
	case  SETUP:
	{
		switch(SETUP_CONFIGURATION)
		{
		case(SETUP_PARAM_DISCHARGE_CURRENT):
		{
			sprintf(LCD_buffer,"Current, mA");
			place = (LCD_COLS-strlen(LCD_buffer))/2;
			LCD_SEND_STR(LCD_buffer,place, 0);

			sprintf(LCD_buffer,"%d",Discharge_current);
			place = (LCD_COLS-strlen(LCD_buffer))/2;
			LCD_SEND_STR(LCD_buffer,place, 1);
			break;
		}
		case(SETUP_PARAM_CUTOFF_VOLTAGE):
		{
			sprintf(LCD_buffer,"Voltage, V");
			place = (LCD_COLS-strlen(LCD_buffer))/2;
			LCD_SEND_STR(LCD_buffer,place, 0);

			sprintf(LCD_buffer,"%d",Cutoff_voltage);
			place = (LCD_COLS-strlen(LCD_buffer))/2;
			LCD_SEND_STR(LCD_buffer,place, 1);
			break;
		}
		default:
			LCD_CLEAR();
			break;
		}
		break;
	}
	case  DISCHARGE:
	{

		break;
	}
	case  FINISH:
	{

		break;
	}
	case  PAUSED:
	{

		break;
	}
	default:
	{

		break;
	}
	}
}
//INTERRUPT CALLBACKS

//EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//test what button is pressed
	if(STATE_MCU != SETUP)
	{
		return;
	}
	switch(GPIO_Pin)
	{
	case(Button_mode_Pin):
					{
		updateScreenRequest = 1;
		SETUP_CONFIGURATION = 	(SETUP_CONFIGURATION+1)%SETUP_PARAM_COUNT;
		break;
					}
	case(Button_add_Pin):
					{

		break;
					}
	case(Button_sub_Pin):
					{

		break;
					}
	default:
	{

		break;
	}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
