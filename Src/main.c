/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t icval_p;
uint32_t icval_f;
uint8_t StartBit;
uint32_t NEC[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t NECT[32] = {1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0};
uint8_t NECi;
uint8_t Decoding;
uint8_t n;
volatile uint8_t tPos;
volatile uint8_t send_code;
double period;

uint32_t Address;
uint32_t Command;
uint32_t ICommand;

volatile uint32_t pTime;
volatile uint32_t cTime;

volatile uint8_t LED_NO;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADDRESS	12288
#define ON	79
#define OFF	78
#define VUP	130
#define VDOWN	131
#define MUTE	20
#define MENU	15
#define BACK	133
#define UP	11
#define DOWN	12
#define LEFT	13
#define RIGHT	14
#define ONE_ARR 8999
#define ONE_CCR 2249
#define ZERO_ARR 4499
#define ZERO_CCR 2249
#define START_ARR 53999
#define START_CCR 35999
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DecodeNec(uint32_t*);
void print_array(uint32_t*);
void comm_to_NECT(uint32_t, uint32_t);
void transmitNECT(void);
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
	icval_p = 0;
	icval_f = 0;
	n = 0;
	period = 0.0;

	StartBit = 0;
	NECi = 0;
	tPos = 0;
	Decoding = 0;
	send_code = 0;

	Address = 0;
	Command = 0;
	ICommand = 0;

	pTime = 0;
	cTime = 0;

	LED_NO = 0;
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Input Capture */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

  	/* LED Turn on OFF Loop */
  	if (LED_NO == 1) {
  		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
  		HAL_Delay(1000);
  		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
  		LED_NO = 0;
  	} else if (LED_NO == 2) {
  		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
			LED_NO = 0;
  	} else if (LED_NO == 3) {
  		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
			LED_NO = 0;
  	} else if (LED_NO == 4) {
  		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
			LED_NO = 0;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 53999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 35999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 216;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 108;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E1_Pin E3_Pin */
  GPIO_InitStruct.Pin = E1_Pin|E3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E4_Pin E2_Pin */
  GPIO_InitStruct.Pin = E4_Pin|E2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Debouncing with 300ms wait */
	cTime = HAL_GetTick();
	if ((cTime - pTime) < 300) {
		return;
	}
	pTime = cTime;

	/* Interrupt Routine */
	if (GPIO_Pin == E1_Pin) {
		comm_to_NECT(ADDRESS, ON);
		transmitNECT();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	} else if (GPIO_Pin == E2_Pin) {
		comm_to_NECT(ADDRESS, OFF);
		transmitNECT();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	} else if (GPIO_Pin == E3_Pin) {
		comm_to_NECT(ADDRESS, VUP);
		transmitNECT();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	} else if (GPIO_Pin == E4_Pin) {
		comm_to_NECT(ADDRESS, VDOWN);
		transmitNECT();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	return;
}


void transmitNECT(void){
	if(send_code == 0) {
		send_code = 1;
		//tPos = 32;

		__HAL_TIM_SET_AUTORELOAD(&htim1, START_ARR);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, START_CCR);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}
}


void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{

	/* If Sending Enabled */
	if(send_code == 1) {
		/* Print Register Values */
		//printf("A[i], i, ARR, CCR1: %d, %d, %d, %d\n", NECT[tPos], tPos, __HAL_TIM_GET_AUTORELOAD(&htim1), __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1));
		//printf("tPos, NECi %d %d\n", tPos, NECi);
		/* Start Sending Data Stream */
		if(NECT[tPos] == 1) {
			/* Pre-load 1 pulse */
			__HAL_TIM_SET_AUTORELOAD(&htim1, ONE_ARR);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ONE_CCR);
			//tPos++;
		} else if (NECT[tPos] == 0) {
			/* Pre-load 0 pulse */
			__HAL_TIM_SET_AUTORELOAD(&htim1, ZERO_ARR);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ZERO_CCR);
			//tPos++;
		}

		/* Disable Interrupt Routine */
		if (tPos == 32) {
			/* Disable PWM Output Routine*/
			send_code = 2; // after next pulse
			return;
		}

		/* Increment NECt Pointer */
		tPos++;

	} else if (send_code == 2) {
		tPos = 0;
		send_code = 0;
		//printf("sc=0 condition, neci %d\n", NECi);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		//HAL_GPIO_WritePin(P41_GPIO_Port, P41_Pin, RESET);
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	}
}


void print_array(uint32_t* array){
	printf("[");
	for(int i = 0; i < 32; i++) {
		if (i == 31) {
			printf("%"PRIu32"]\n", array[i]);
		} else {
			printf("%"PRIu32" ", array[i]);
		}
	}
}

void comm_to_NECT(uint32_t add, uint32_t comm){
	int k;
	for(int i = 0; i < 16; i++){
		k = add >> i;
		if (k & 1) {
			NECT[i] = 1;
		} else {
			NECT[i] = 0;
		}
	}
	for(int i = 0; i < 8; i++){
		k = comm >> i;
		if (k & 1) {
			NECT[16+i] = 1;
			NECT[24+i] = 0;
		} else {
			NECT[16+i] = 0;
			NECT[24+i] = 1;
		}
	}
}

void DMATransferComplete(DMA_HandleTypeDef *hdma)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	/* Read Counter Vals */
	if(htim->Instance == TIM2) {
		icval_f = __HAL_TIM_GET_COUNTER(htim);
		__HAL_TIM_SET_COUNTER(htim, 0);
	}

	period = 1000.0*icval_f/(8000000.0/65.0);
	icval_p = icval_f;
	//printf("period: %e\n", period);

	if (1){
		/* Searching for Start Bit */
		if(StartBit == 0) {
			if ((13 <= period) && (period <= 14)) {
				StartBit = 1;
				NECi = 0;
			}
			// This else if doesn't account for what happens on multiple transmissions
			// Starbit == 1 remains 1 even after transmission, but should it be 0 after transmission?
		} else if((StartBit == 1) && (Decoding == 0)) {
			//printf("NECi %d\n", NECi);
			if ((2 <= period) && (period <= 2.5)) {
				NEC[NECi++] = 1;
			} else if ((1 <= period) && (period <= 1.8)) {
				NEC[NECi++] = 0;
			}
			if (NECi == 32) {
				NECi = 0;
				Decoding = 1;
				DecodeNec(NEC);
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
		}
	}
	return;
}

void DecodeNec(uint32_t* Array) {

		/* Read Address */
		for (int i = 0; i < 16; i++) {
				Address += Array[i]*pow(2, i);
		}

		/* Read Command */
		for (int i = 16; i < 24; i++) {
			Command += Array[i]*pow(2, (i-16));
		}

		/* Read Inverse Command */
		for (int i = 24; i < 32; i++) {
			ICommand += (Array[i]^1)*pow( 2, (i-24));
		}

		print_array(Array);
		printf("Address: %"PRIu32", Command: %"PRIu32", ICommand: %"PRIu32"\n", Address, Command, ICommand);

		/* Determine What Happens */
		switch(Command) {
		case ON:
			LED_NO = 1;
			break;
		case OFF:
			LED_NO = 2;
			break;
		case VUP:
			LED_NO = 3;
			break;
		case VDOWN:
			LED_NO = 4;
			break;
		}

		/* Reset Array */
		for (int i = 0; i < 32; i++) {
			Array[i] = 0;
		}

		/* Finished Decoding for NEC protocol, allow for more NEC transmissions */
	  Address = 0;
	  Command = 0;
	  ICommand = 0;
		Decoding = 0;
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
