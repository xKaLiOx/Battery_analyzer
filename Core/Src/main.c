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
#define BUTTON_DEBOUNCE_MS 30//30 ms for debouncing
#define BUTTON_LONG_PRESS 1000//2s long press

#define LCD_BUFFER_SIZE 32
#define R_load 0.22 //load ohms
#define R_divider_multiplier 1/(50/270.0f) // 50k/270k voltage divider to calculate battery voltage
#define Vrefint 1.2 //1.2V internal reference voltage
#define OVERCURRENT_VALUE 2000 // 2 A trip current
#define ADC_steps 4096


#define ADC_DMA_SIZE 100 //0.5ms sampling rate, 25 values on half callback, 2 channels, ping pong buffer
#define ADC_TRIGGER_FREQ_HZ 2000.0f //counter update rate
#define ADC_SAMPLES_PER_CHANNEL 25.0f
#define TIME_SLICE_S ADC_SAMPLES_PER_CHANNEL/ADC_TRIGGER_FREQ_HZ
#define MAH_CONVERSION (TIME_SLICE_S/3600.0f)
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
volatile uint8_t button_activity = 0;
volatile uint8_t OverCurrent_indication = 0;
volatile uint32_t last_button_time = 0;

volatile uint16_t Discharge_current = 500;//mA
volatile uint16_t Cutoff_voltage = 3000;//mV

volatile uint32_t ADC_VOLTAGE_ACCUM = 0;
volatile uint32_t ADC_CURRENT_ACCUM = 0;
volatile uint16_t ADC_READING_COUNTER = 0;
volatile DischargeDisplayData_t DischargeDisplayData = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int le, char *ptr, int len);
void DELAY_US(uint16_t TIME_US);
void charAddPadding(char* buffer, uint8_t align,uint8_t size);
void updateScreen();
void formatCharToLCD(char* message, uint8_t place, uint8_t level, uint8_t Padding);
void PWM_Control_loop(TIM_HandleTypeDef *htim, uint16_t *current, uint16_t voltage);
uint16_t CurrentToVoltage(uint32_t Shunt_voltage);//voltage on sense resistor
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
	static uint32_t last_tick;
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
	HAL_GPIO_WritePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin,GPIO_PIN_SET);// OP AMP inv input to turn off discharge


	if(HAL_ADCEx_Calibration_Start(&hadc1) !=HAL_OK)
	{
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}
	if(HAL_TIM_Base_Start_IT(&htim4)!=HAL_OK)//TIM2 for DELAY_US
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
	LCD_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(updateScreenRequest)//only perform LCD switch states on gpio change
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
			//ONCE TO DO
			if(STATE_MCU_CURRENT != STATE_MCU_PREVIOUS)
			{
				DischargeDisplayData.capacity_mah = 0;

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
				if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4)!=HAL_OK)//TIM2 for MOSFET PWM
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				STATE_MCU_PREVIOUS = STATE_MCU_CURRENT;
				DischargeDisplayData.start_time = uwTick;
				HAL_GPIO_WritePin(CHARGING_STATE_GPIO_Port, CHARGING_STATE_Pin,GPIO_PIN_RESET); // indication for charging
				HAL_GPIO_WritePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin,GPIO_PIN_RESET);
				last_tick = uwTick;
				break;
			}



			//PERIODIC UPDATE
			if(uwTick - last_tick > 500)// 0.5s refresh rate
			{
				updateScreenRequest=1;
				uint32_t local_temp_volt;
				uint32_t local_temp_curr;
				uint16_t local_temp_count;

				__disable_irq();
				local_temp_volt = ADC_VOLTAGE_ACCUM;
				local_temp_curr = ADC_CURRENT_ACCUM;
				local_temp_count = ADC_READING_COUNTER;

				ADC_READING_COUNTER=0;
				ADC_CURRENT_ACCUM=0;
				ADC_VOLTAGE_ACCUM=0;
				__enable_irq();

				DischargeDisplayData.voltage = Vdda*local_temp_volt/local_temp_count/ADC_steps*R_divider_multiplier;//convert from ADC to Voltage NOT CORRECT
				DischargeDisplayData.current_ma = Vdda*local_temp_curr/local_temp_count/R_load/ADC_steps*1000;//curr is voltage(XD)

				last_tick = uwTick;

				if(HAL_GPIO_ReadPin(Button_add_GPIO_Port, Button_add_Pin)==GPIO_PIN_SET)
				{
					TIM2->CCR4 += 10;
				}

				if(HAL_GPIO_ReadPin(Button_sub_GPIO_Port, Button_sub_Pin)==GPIO_PIN_SET)
				{
					TIM2->CCR4 -= 10;
				}
			}
			break;
		}
		case  FINISH:
		{
			if(STATE_MCU_CURRENT != STATE_MCU_PREVIOUS)
			{
				if(HAL_TIM_Base_Stop_IT(&htim3)!=HAL_OK)//TIM3 for SAMPLING
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				if(HAL_ADC_Stop_DMA(&hadc1)!=HAL_OK)//ADC SAMPLING DMA
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4)!=HAL_OK)//TIM2 for MOSFET PWM
				{
					HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
					Error_Handler();
				}
				//STATE_MCU_PREVIOUS = STATE_MCU_CURRENT; IMPLEMENTED IN BUTTON
				HAL_GPIO_WritePin(CHARGING_STATE_GPIO_Port, CHARGING_STATE_Pin,GPIO_PIN_SET); // indication for charging
				HAL_GPIO_WritePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin,GPIO_PIN_RESET);
				last_tick = uwTick;
				break;
			}
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

		if(button_activity)
		{
			if(HAL_GPIO_ReadPin(Button_mode_GPIO_Port, Button_mode_Pin) == GPIO_PIN_SET)
			{
				if(STATE_MCU_CURRENT == SETUP)
				{
					SETUP_CONFIGURATION = 	(SETUP_CONFIGURATION+1)%SETUP_PARAM_COUNT;
					while(HAL_GPIO_ReadPin(Button_mode_GPIO_Port, Button_mode_Pin) == GPIO_PIN_SET)
					{
						if(uwTick-last_button_time > BUTTON_LONG_PRESS)
						{
							STATE_MCU_CURRENT = DISCHARGE;
							STATE_MCU_PREVIOUS = SETUP;
							last_button_time = uwTick;
							break;//stop the loop, since its in different state now
						}
					}
				}

				if (STATE_MCU_CURRENT == DISCHARGE)
				{
						HAL_GPIO_TogglePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin);
				}
				if(STATE_MCU_CURRENT == FINISH)
				{
					//reset for setup to start over
					STATE_MCU_CURRENT = SETUP;
					STATE_MCU_PREVIOUS = FINISH;
				}
			}

			else if(HAL_GPIO_ReadPin(Button_add_GPIO_Port, Button_add_Pin) == GPIO_PIN_SET)
			{
				if(STATE_MCU_CURRENT == SETUP)
				{
					if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
					{
						Cutoff_voltage += 100;// 100 mV step
					}
					else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
					{
						Discharge_current+=10;// 10 mA step
					}
					while(HAL_GPIO_ReadPin(Button_add_GPIO_Port, Button_add_Pin) == GPIO_PIN_SET)
					{
						if(uwTick-last_button_time > BUTTON_LONG_PRESS)
						{
							if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
							{
								Cutoff_voltage += 100;// 100 mV step
							}
							else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
							{
								Discharge_current+=10;// 10 mA step
							}
							DELAY_US(10000);
							updateScreen();//periodically update because of the auto increment
						}
					}
					last_button_time = uwTick;
				}
			}

			else if(HAL_GPIO_ReadPin(Button_sub_GPIO_Port, Button_sub_Pin) == GPIO_PIN_SET)
			{
				if(STATE_MCU_CURRENT == SETUP)
				{
					if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
					{
						Cutoff_voltage -= 100;// 100 mV step
					}
					else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
					{
						Discharge_current-=10;// 10 mA step
					}
					while(HAL_GPIO_ReadPin(Button_sub_GPIO_Port, Button_sub_Pin) == GPIO_PIN_SET)
					{
						if(uwTick-last_button_time > BUTTON_LONG_PRESS)
						{
							if(SETUP_CONFIGURATION == SETUP_PARAM_CUTOFF_VOLTAGE)
							{
								Cutoff_voltage -= 100;// 100 mV step
							}
							else if(SETUP_CONFIGURATION == SETUP_PARAM_DISCHARGE_CURRENT)
							{
								Discharge_current-=10;// 10 mA step
							}
							DELAY_US(10000);
							updateScreen();//periodically update because of the auto increment
						}
					}
					last_button_time = uwTick;
				}
			}
			button_activity = 0;
			updateScreenRequest = 1;
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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
	uint32_t old_timer_value = TIM4->CNT;
	uint32_t target_time = (old_timer_value + TIME_US) % (TIM4->ARR + 1);

	if (target_time < old_timer_value)  // Handle timer overflow
	{
		while (TIM4->CNT >= old_timer_value);  // Wait for overflow
	}
	while (TIM4->CNT < target_time);  // Wait until target time is reached
}
void updateScreen()
{
	static uint32_t elapsed_time;
	updateScreenRequest = 0;
	switch(STATE_MCU_CURRENT)
	{
	case  START:
	{
		LCD_Init();

		sprintf(LCD_buffer,"Battery analyzer");
		formatCharToLCD(LCD_buffer,0,0,ALIGN_LEFT);

		sprintf(LCD_buffer,"BatVinux");
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
			sprintf(LCD_buffer,"Cutoff voltage");
			formatCharToLCD(LCD_buffer,0,0,ALIGN_CENTER);

			//no float support (+10 kB flash)
			uint8_t temp1 = Cutoff_voltage/1000;
			uint8_t temp2 = (Cutoff_voltage/100)%10;
			sprintf(LCD_buffer,"%u.%u V",temp1,temp2);
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
		if(STATE_MCU_CURRENT != STATE_MCU_PREVIOUS)
		{
			sprintf(LCD_buffer,"Starting");
			formatCharToLCD(LCD_buffer,0,0,ALIGN_CENTER);

			sprintf(LCD_buffer,"the discharge...");
			formatCharToLCD(LCD_buffer,0,1,ALIGN_CENTER);

			HAL_Delay(2000);
			break;
		}
		if(OverCurrent_indication != 1)
		{
		//Printing the reading values
		uint16_t temp = (uint16_t)(((10*DischargeDisplayData.voltage)+0.5))%10;
		char buff[8];
		sprintf(buff,"%u.%u",(uint16_t)DischargeDisplayData.voltage,temp);
		sprintf(LCD_buffer,"%u mA, %s V",(uint16_t)(DischargeDisplayData.current_ma+0.5),buff);
		formatCharToLCD(LCD_buffer,0,0,ALIGN_LEFT);

		elapsed_time = (uwTick-DischargeDisplayData.start_time)/1000;
		sprintf(LCD_buffer,"%lu mAh, %lu s",(uint32_t)DischargeDisplayData.capacity_mah,elapsed_time);
		formatCharToLCD(LCD_buffer,0,1,ALIGN_LEFT);
		}
		else
		{
			sprintf(LCD_buffer,"OVERCURRENT");
			formatCharToLCD(LCD_buffer,0,0,ALIGN_LEFT);

			sprintf(LCD_buffer,"CHECK BATTERY");
			formatCharToLCD(LCD_buffer,1,0,ALIGN_LEFT);

			OverCurrent_indication = 0;
		}

		break;
	}
	case  FINISH:
	{
		sprintf(LCD_buffer,"DISCHARGED");
		formatCharToLCD(LCD_buffer,0,1,ALIGN_CENTER);


		uint32_t elapsed_time = (uwTick-DischargeDisplayData.start_time)/1000;
		sprintf(LCD_buffer,"%u mAh in %lu s?",(uint16_t)DischargeDisplayData.capacity_mah,elapsed_time);
		formatCharToLCD(LCD_buffer,0,1,ALIGN_RIGHT);

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

void PWM_Control_loop(TIM_HandleTypeDef *htim, uint16_t *current, uint16_t voltage)
{
	if(*current > OVERCURRENT_VALUE)//overcurrent protection 2.5A
	{
		HAL_GPIO_WritePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin,GPIO_PIN_SET);
		*current = 0;
		OverCurrent_indication = 1;
	}

	if(Cutoff_voltage < (voltage*1000))
	{
		HAL_GPIO_WritePin(DISCHARGE_STOP_GPIO_Port, DISCHARGE_STOP_Pin,GPIO_PIN_SET);
		STATE_MCU_CURRENT = FINISH;
		STATE_MCU_PREVIOUS = DISCHARGE;
	}
}

//INTERRUPT CALLBACKS

//EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(uwTick-last_button_time > BUTTON_DEBOUNCE_MS)
	{
		button_activity = 1;
		last_button_time = uwTick;
	}

	//IMPLEMENT THE STARTING PROCESS
	//	STATE_MCU_CURRENT = DISCHARGE;
	//	STATE_MCU_PREVIOUS = SETUP;

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_sum[2] = {0};
	static uint16_t Current_BATT;
	static uint16_t Voltage; // for cutoff calculation only


	//AVERAGING
	for(uint16_t i = 0; i < (ADC_DMA_SIZE/2)-1; i=i+2)
	{
		adc_sum[0] += ADC_Values[i];//first channel
		adc_sum[1] += ADC_Values[i+1];//second channel
	}
	adc_sum[0] = 4*adc_sum[0]/ADC_DMA_SIZE;
	adc_sum[1] = 4*adc_sum[1]/ADC_DMA_SIZE;


	//CURRENT AND MAH CONVERSION
	Current_BATT = 3300*adc_sum[1]/R_load/ADC_steps;//convert to mA
	Voltage = (uint16_t)((Vdda*adc_sum[0])*1.0/ADC_steps*R_divider_multiplier);//in volts
	PWM_Control_loop(&htim2,&Current_BATT,Voltage);
	DischargeDisplayData.capacity_mah += Current_BATT*MAH_CONVERSION;

	//defensive guard band
	__disable_irq();
	ADC_VOLTAGE_ACCUM += adc_sum[0];
	ADC_CURRENT_ACCUM += adc_sum[1];
	ADC_READING_COUNTER+=1;
	__enable_irq();

	//printf("HalfCallback %lu",uwTick);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_sum[2] = {0};
	static uint16_t Current_BATT;
	static uint16_t Voltage; // for cutoff calculation only

	for(uint16_t i = (ADC_DMA_SIZE/2); i < ADC_DMA_SIZE; i=i+2)
	{
		adc_sum[0] += ADC_Values[i];//first channel
		adc_sum[1] += ADC_Values[i+1];//second channel
	}
	//AVERAGES
	adc_sum[0] = 4*adc_sum[0]/ADC_DMA_SIZE;
	adc_sum[1] = 4*adc_sum[1]/ADC_DMA_SIZE;

	//CURRENT AND MAH CONVERSION
	Current_BATT = 3300*adc_sum[1]/R_load/ADC_steps;//convert to mA
	Voltage = (uint16_t)Vdda*adc_sum[0]/ADC_steps*R_divider_multiplier;//in volts
	PWM_Control_loop(&htim2,&Current_BATT,Voltage);
	DischargeDisplayData.capacity_mah += Current_BATT*MAH_CONVERSION;

	//defensive guard band
	__disable_irq();
	ADC_VOLTAGE_ACCUM += adc_sum[0];
	ADC_CURRENT_ACCUM += adc_sum[1];
	ADC_READING_COUNTER+=1;
	__enable_irq();

	//PWM CONTROL LOOP
	//printf("FullCallback %lu",uwTick);
}

int _write(int le, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
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
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
		HAL_Delay(200);
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
