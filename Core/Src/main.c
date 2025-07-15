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
#include "dma.h"
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
#define ALIGN_NO 0
#define ALIGN_LEFT 1
#define ALIGN_RIGHT 2
#define ALIGN_CENTER 3

#define LCD_BUFFER_SIZE 32
#define R_load 0.22 //load ohms
#define Vrefint 1.2 //1.2V internal reference voltage
#define ADC_steps 4096


#define ADC_DMA_SIZE 100 //0.5ms sampling rate, 25 values on half callback, 2 channels, ping pong buffer
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
volatile FSM_states STATE_MCU_CURRENT = START;
volatile FSM_states STATE_MCU_PREVIOUS = START;
volatile SETUP_set SETUP_CONFIGURATION = SETUP_PARAM_DISCHARGE_CURRENT;
char LCD_buffer[LCD_BUFFER_SIZE];
uint16_t ADC_Values[ADC_DMA_SIZE];//ping pong buffer
volatile float Vdda = 3.3; //ref for measurements, calculated later by ADC with internal ref
volatile uint8_t updateScreenRequest = 1;//update at setup on first frame

volatile uint16_t Discharge_current = 500;//mA
volatile float Cutoff_voltage = 2.5;//V
volatile float ADC_VOLTAGES_MEAN[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DELAY_US(uint16_t TIME_US);
void charAddPadding(char* buffer, uint8_t align,uint8_t size);
void updateScreen();
void formatCharToLCD(char* message, uint8_t place, uint8_t level, uint8_t Padding);

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	if(HAL_ADCEx_Calibration_Start(&hadc1) !=HAL_OK)
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}

	if(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK)//TIM2 for DELAY_US
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}

	HAL_Delay(500);// wait for DC point

	//Read internal reference for VDDA
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, 500);
	uint16_t Vadc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	Vdda = 1.0*ADC_steps/Vadc*Vrefint;

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

		switch(STATE_MCU_CURRENT)
		{
		case  SETUP:
		{
			break;
		}
		case  DISCHARGE:
		{
			if(STATE_MCU_CURRENT != STATE_MCU_PREVIOUS)
			{
				if(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK)//TIM3 for SAMPLING
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_DMA_SIZE)!=HAL_OK)//ADC SAMPLING DMA
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)!=HAL_OK)//TIM4 for MOSFET PWM
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				STATE_MCU_PREVIOUS = STATE_MCU_CURRENT;
			}
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
	uint32_t old_timer_value = TIM2->CNT;
	uint32_t target_time = (old_timer_value + TIME_US) % (TIM2->ARR + 1);

	if (target_time < old_timer_value)  // Handle timer overflow
	{
		while (TIM2->CNT >= old_timer_value);  // Wait for overflow
	}

	while (TIM2->CNT < target_time);  // Wait until target time is reached
}
void updateScreen()
{
	updateScreenRequest = 0;
	switch(STATE_MCU_CURRENT)
	{
	case  START:
	{
		LCD_Init();

		sprintf(LCD_buffer,"Battery analyzer");
		formatCharToLCD(LCD_buffer,0,0,ALIGN_LEFT);

		sprintf(LCD_buffer,"BatLinux");
		formatCharToLCD(LCD_buffer,0,1,ALIGN_CENTER);

		HAL_Delay(1000);
		STATE_MCU_CURRENT = SETUP;
		break;
	}
	case  SETUP:
	{
		switch(SETUP_CONFIGURATION)
		{
		case(SETUP_PARAM_DISCHARGE_CURRENT):
					{
			sprintf(LCD_buffer,"Current, mA");
			formatCharToLCD(LCD_buffer,0,0,ALIGN_CENTER);

			sprintf(LCD_buffer,"%d",Discharge_current);
			formatCharToLCD(LCD_buffer,0,1,ALIGN_CENTER);
			break;
					}
		case(SETUP_PARAM_CUTOFF_VOLTAGE):
					{
			sprintf(LCD_buffer,"Voltage, V");
			formatCharToLCD(LCD_buffer,0,0,ALIGN_CENTER);

			//no float support (+10 kB flash)
			uint8_t separator = 10*Cutoff_voltage-10*(int)Cutoff_voltage;
			sprintf(LCD_buffer,"%d.%d",(int)Cutoff_voltage,separator);
			formatCharToLCD(LCD_buffer,1,1,ALIGN_CENTER);
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
//Add padding for LCD display
//If padding is added, place is 0, level is dependant on LCD top(0) or bottom(1)
void formatCharToLCD(char* message, uint8_t place, uint8_t level, uint8_t Padding)
{
	static uint8_t buffer_size;
	buffer_size = strlen(message);
	if(buffer_size != 0 && buffer_size <= LCD_COLS)
	{
		if(Padding==ALIGN_NO)
		{
			LCD_SEND_STR(message,place, level);
		}
		else if(Padding==ALIGN_LEFT)
		{
			charAddPadding(message, ALIGN_LEFT, buffer_size);
			LCD_SEND_STR(message,0, level);
		}
		else if(Padding==ALIGN_RIGHT)
		{
			charAddPadding(message, ALIGN_RIGHT, buffer_size);
			LCD_SEND_STR(message,0, level);
		}
		else if(Padding==ALIGN_CENTER)
		{
			charAddPadding(message, ALIGN_CENTER, buffer_size);
			LCD_SEND_STR(message,0, level);
		}
		else return;
	}
}

void charAddPadding(char* buffer, uint8_t align,uint8_t size)
{
	if(align == ALIGN_LEFT)
	{
		for(uint8_t i = size;i<LCD_COLS;i++)
		{
			buffer[i] = ' ';
		}
	}
	else if(align == ALIGN_RIGHT)
	{
		memmove(buffer+(LCD_COLS-size),buffer,size);// shift to the right
		for(uint8_t i = 0;i<LCD_COLS-size;i++)
		{
			buffer[i] = ' ';
		}
	}
	else if(align == ALIGN_CENTER)
	{
		uint8_t start_place = (LCD_COLS - size)/2;//left side
		memmove(buffer+start_place,buffer,size);// shift to the right
		for(uint8_t i = 0;i<start_place;i++)
		{
			buffer[i] = ' ';
		}
		start_place +=size;//right side
		for(uint8_t i = start_place;i<LCD_COLS;i++)
		{
			buffer[i] = ' ';
		}
	}
	else
	{
		return;
	}
}

//INTERRUPT CALLBACKS

//EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//test what button is pressed
	//IMPLEMENT THE STARTING PROCESS
	STATE_MCU_CURRENT = DISCHARGE;
	STATE_MCU_PREVIOUS = SETUP;
	if(STATE_MCU_CURRENT != SETUP)
	{
		return;
	}
	updateScreenRequest = 1;
	switch(GPIO_Pin)
	{
	case(Button_mode_Pin):
			{
		SETUP_CONFIGURATION = 	(SETUP_CONFIGURATION+1)%SETUP_PARAM_COUNT;
		break;
			}
	case(Button_add_Pin):
					{
		if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
		{
			Cutoff_voltage += 0.05;// 50 mV step
		}
		else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
		{
			Discharge_current+=10;// 10 mA step
		}
		break;
					}
	case(Button_sub_Pin):
			{
		if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
		{
			Cutoff_voltage -= 0.05;// 50 mV step
		}
		else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
		{
			Discharge_current-=10;// 10 mA step
		}
		break;
			}
	default:
	{

		break;
	}
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_sum[2] = {0};

	for(uint16_t i = 0; i < (ADC_DMA_SIZE/2)-1; i=i+2)
	{
		adc_sum[0] += ADC_Values[i];//first channel
		adc_sum[1] += ADC_Values[i+1];//second channel
	}

	ADC_VOLTAGES_MEAN[0] = 4.0*(float)adc_sum[0]/ADC_DMA_SIZE;
	ADC_VOLTAGES_MEAN[1] = 4.0*(float)adc_sum[1]/(ADC_DMA_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_sum[2] = {0};

	for(uint16_t i = (ADC_DMA_SIZE/2); i < ADC_DMA_SIZE; i=i+2)
	{
		adc_sum[0] += ADC_Values[i];//first channel
		adc_sum[1] += ADC_Values[i+1];//second channel
	}
	ADC_VOLTAGES_MEAN[0] = 4.0*(float)adc_sum[0]/ADC_DMA_SIZE;
	ADC_VOLTAGES_MEAN[1] = 4.0*(float)adc_sum[1]/(ADC_DMA_SIZE);
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
