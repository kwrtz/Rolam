/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

// Wert von  htim2.Init.Period = 1185; in timer 2 kann dazu verwendet werden,
// die peakdistanz auf 480 zu justieren
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "calculation.h"
#include "cmd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t convCompletedL = 0;
volatile uint8_t convCompletedR = 0;
volatile uint8_t convCompletedB = 0;

// Arrays for adc values
int16_t adcValueL[EMPF_ARRAY_SIZE];
int16_t adcValueR[EMPF_ARRAY_SIZE];
int16_t adcValueB[EMPF_ARRAY_SIZE];

// Result values for each calculated coil. Includes also a pointer to current adcValueX.
CAL_ChannelTypeDef coilL;
CAL_ChannelTypeDef coilR;
CAL_ChannelTypeDef coilB;

// Flags will be set by commandfunctions and determine if a printfunction is called.
bool flagShowSignalL = false;
bool flagShowSignalR = false;
bool flagShowSignalB = false;
bool flagShowADCSignalL = false;
bool flagShowADCSignalR = false;
bool flagShowADCSignalB = false;
bool flagShowADCSignalLRaw = false;
bool flagShowADCSignalRRaw = false;
bool flagShowADCSignalBRaw = false;
bool flag_ShowValuesSendToMaster = false;
bool flagShowOffset = false;
bool flagShowLoopTime = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void cmd_help(int arg_cnt, char **args);
void cmd_showSignalL(int arg_cnt, char **args);
void cmd_showSignalR(int arg_cnt, char **args);
void cmd_showSignalB(int arg_cnt, char **args);
void cmd_showADCSignalL(int arg_cnt, char **args);
void cmd_showADCSignalR(int arg_cnt, char **args);
void cmd_showADCSignalB(int arg_cnt, char **args);
void cmd_showADCSignalLRaw(int arg_cnt, char **args);
void cmd_showADCSignalRRaw(int arg_cnt, char **args);
void cmd_showADCSignalBRaw(int arg_cnt, char **args);
void cmd_showValuesSendToDue(int arg_cnt, char **args);
void cmd_showOffset(int arg_cnt, char **args);
void cmd_showLoopTime(int arg_cnt, char **args);
void cmd_hideValues(int arg_cnt, char **args);
void cmd_showFreeMem(int arg_cnt, char **args);

extern void sendToMaster(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR, CAL_ChannelTypeDef *confB,
		bool flag_ShowValuesSendToDue);
extern void ledSetState(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR);
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	char msg[40];
	volatile uint32_t time, deltaTime;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_ADC2_Init();
	MX_USART3_UART_Init();
	MX_ADC3_Init();

	/* USER CODE BEGIN 2 */

	//HAL_TIM_Base_Start_IT(&htim2); //Use for checking timer frequency on D7
	HAL_TIM_Base_Start(&htim2);
	cal_init();
	cmdInit(); // Activates also USART2 Interrupt

	cmdAdd((char *) ("H"), cmd_help);
	cmdAdd((char *) ("l"), cmd_showSignalL);
	cmdAdd((char *) ("r"), cmd_showSignalR);
	cmdAdd((char *) ("b"), cmd_showSignalB);
	cmdAdd((char *) ("adcl"), cmd_showADCSignalL);
	cmdAdd((char *) ("adcr"), cmd_showADCSignalR);
	cmdAdd((char *) ("adcb"), cmd_showADCSignalB);
	cmdAdd((char *) ("adclraw"), cmd_showADCSignalLRaw);
	cmdAdd((char *) ("adcrraw"), cmd_showADCSignalRRaw);
	cmdAdd((char *) ("adcbraw"), cmd_showADCSignalBRaw);
	cmdAdd((char *) ("h"), cmd_hideValues);
	cmdAdd((char *) ("H"), cmd_help);
	cmdAdd((char *) ("s"), cmd_showValuesSendToDue);
	cmdAdd((char *) ("o"), cmd_showOffset);
	cmdAdd((char *) ("t"), cmd_showLoopTime);
	cmdAdd((char *) ("f"), cmd_showFreeMem);

	coilL.empfangssignal = adcValueL;
	coilR.empfangssignal = adcValueR;
	coilB.empfangssignal = adcValueB;

	convCompletedL = 0;
	convCompletedR = 0;
	convCompletedB = 0;

	// Linke Spule samplen
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcValueL, EMPF_ARRAY_SIZE) != HAL_OK) {
		// Start Conversation Error
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		time = HAL_GetTick();

		//-----------------------------------------
		// Rechte Spule samplen
		//-----------------------------------------
		if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adcValueR, EMPF_ARRAY_SIZE) != HAL_OK) {
			// Start Conversation Error
			Error_Handler();
		}

		//-----------------------------------------
		// Linke Spule berechnen
		//-----------------------------------------
		while (!convCompletedL)
			// Wait for sample rechte spule to finish
			;
		convCompletedL = 0;
		HAL_Delay(1); //Wait one ms that the second dma channel finishing datatransfer. If not waiting, the last value in the array is wrong.
		HAL_ADC_Stop_DMA(&hadc1);

		// Print adc raw signal
		if (flagShowADCSignalLRaw) {
			cal_printADCSignal(&coilL, 50000, &flagShowADCSignalLRaw);  //Empfangssignal auf Konsole ausgeben
		}

		cal_offset(&coilL);
		cal_convolve(&coilL);

		if (flagShowSignalL) {
			cal_printSignal(&coilL, 50000, &flagShowSignalL);  //Empfangssignal und Korrelatiossignal auf Konsole ausgeben
		}
		if (flagShowADCSignalL) {
			cal_printADCSignal(&coilL, 50000, &flagShowADCSignalL);  //Empfangssignal auf Konsole ausgeben
		}

		//-----------------------------------------
		// Hintere Spule samplen
		//-----------------------------------------
		if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adcValueB, EMPF_ARRAY_SIZE) != HAL_OK) {
			// Start Conversation Error
			Error_Handler();
		}

		//-----------------------------------------
		// Recht Spule berechnen
		//-----------------------------------------
		while (!convCompletedR)
			// Wait for sample rechte spule to finish
			;
		convCompletedR = 0;
		HAL_Delay(1); //Wait one ms that the second dma channel finishing datatransfer. If not waiting, the last value in the array is wrong.
		HAL_ADC_Stop_DMA(&hadc2);

		if (flagShowADCSignalRRaw) {
			cal_printADCSignal(&coilR, 40000, &flagShowADCSignalRRaw);  //Empfangssignal auf Konsole ausgeben
		}

		cal_offset(&coilR);
		cal_convolve(&coilR);

		if (flagShowSignalR) {
			cal_printSignal(&coilR, 40000, &flagShowSignalR);  //Empfangssignal und Korrelatiossignal auf Konsole ausgeben
		}
		if (flagShowADCSignalR) {
			cal_printADCSignal(&coilR, 40000, &flagShowADCSignalR);  //Empfangssignal auf Konsole ausgeben
		}
		//-----------------------------------------
		// Linke Spule samplen
		//-----------------------------------------
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcValueL, EMPF_ARRAY_SIZE) != HAL_OK) {
			// Start Conversation Error
			Error_Handler();
		}

		//-----------------------------------------
		// Hintere Spule berechnen
		//-----------------------------------------
		while (!convCompletedB)
			// Wait for sample rechte spule to finish
			;
		convCompletedB = 0;
		HAL_Delay(1); //Wait one ms that the second dma channel finishing datatransfer. If not waiting, the last value in the array is wrong.
		HAL_ADC_Stop_DMA(&hadc3);

		if (flagShowADCSignalBRaw) {
			cal_printADCSignal(&coilB, 30000, &flagShowADCSignalBRaw);  //Empfangssignal auf Konsole ausgeben
		}

		cal_offset(&coilB);
		cal_convolve(&coilB);

		if (flagShowSignalB) {
			cal_printSignal(&coilB, 30000, &flagShowSignalB);  //Empfangssignal und Korrelatiossignal auf Konsole ausgeben
		}
		if (flagShowADCSignalB) {
			cal_printADCSignal(&coilB, 30000, &flagShowADCSignalB);  //Empfangssignal auf Konsole ausgeben
		}



		//-----------------------------------------
		// An Master senden
		//-----------------------------------------
		sendToMaster(&coilL, &coilR, &coilB, flag_ShowValuesSendToMaster); // Sets LED status too
		ledSetState(&coilL, &coilR);


		//-----------------------------------------
		// Offset ausgeben
		//-----------------------------------------

		if (flagShowOffset) {
			cal_printOffset(&coilL, &coilR, &coilB);
		}

		//-----------------------------------------
		// Loopdauer ausgeben
		//-----------------------------------------

		if (flagShowLoopTime) {
			deltaTime = HAL_GetTick() - time;
			time = HAL_GetTick();

			sprintf(msg, "time: %lu\r\n", time);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);

			sprintf(msg, "deltaTime: %lu\r\n", deltaTime);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);

		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* ADC2 init function */
static void MX_ADC2_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* ADC3 init function */
static void MX_ADC3_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1185;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/* USART3 init function */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 19200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC2 PC3 PC4 PC6
	 PC7 PC8 PC9 PC10
	 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9
			| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 PA4 PA9 PA10
	 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin LDLEFT_Pin LDRIGHT_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | LDLEFT_Pin | LDRIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB12
	 PB13 PB14 PB15 PB4
	 PB5 PB6 PB7 PB8
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
			| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : FREQADCOUT_Pin */
	GPIO_InitStruct.Pin = FREQADCOUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FREQADCOUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | LDLEFT_Pin | LDRIGHT_Pin | FREQADCOUT_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */


/* Using malloc() to determine free memory.
 Hi Ed, I have found a little program to know how much RAM is available for dynamic allocation.
 I call FreeMem() at several critical places to understand what happens with the memory allocation.
 The code is short, I hope this can help you.
 Robert
 */
#define FREEMEM_CELL 100
struct elem { /* Definition of a structure that is FREEMEM_CELL bytes  in size.) */
	struct elem *next;
	char dummy[FREEMEM_CELL - 4]; // für 32 Bit Prozessor
//char dummy[FREEMEM_CELL-2]; // für  8 Bit Prozessor
};
unsigned long FreeMem(void) {
	unsigned long counter;
	struct elem *head, *current, *nextone;
	current = head = (struct elem*) malloc(sizeof(struct elem));
	if (head == NULL)
		return 0; /*No memory available.*/
	counter = 0;
	// __disable_irq();
	do {
		counter++;
		current->next = (struct elem*) malloc(sizeof(struct elem));
		current = current->next;
	} while (current != NULL);
	/* Now counter holds the number of type elem
	 structures we were able to allocate. We
	 must free them all before returning. */
	current = head;
	do {
		nextone = current->next;
		free(current);
		current = nextone;
	} while (nextone != NULL);
	// __enable_irq();

	return counter * FREEMEM_CELL;
}

//--------------------------------------------------
// User interface functions
//--------------------------------------------------
const char cmd_help_string[] = "\r\n"
		"l    show signals left\r\n"
		"r    show signals right\r\n"
		"b    show signals back\r\n"
		"adcl show adc values left corrected by offset\r\n"
		"adcr show adc values right corrected by offset\r\n"
		"adcb show adc values back corrected by offset\r\n"
		"adclraw show captured adc values left \r\n"
		"adcrraw show captured adc values right\r\n"
		"adcbraw show captured adc values back\r\n"
		"o    show offset of ADC signal\r\n"
		"s    show values sending to due + quality and peak distance\r\n"
		"h    stop showing\r\n"
		"H    will print this help message again\r\n"
		"t    show loop duration\r\n"
		"f    show free mem\r\n"
		"\r\n";

void cmd_help(int arg_cnt, char **args) {
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd_help_string, strlen(cmd_help_string), HAL_MAX_DELAY);
}

void cmd_showFreeMem(int arg_cnt, char **args) {
	char msg[100];
	unsigned long i = FreeMem();
	sprintf(msg, "FreeMem: %lu\r\n", i);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(10);
}

void cmd_showSignalL(int arg_cnt, char **args) {
	flagShowSignalL = true;
}

void cmd_showSignalR(int arg_cnt, char **args) {
	flagShowSignalR = true;
}

void cmd_showSignalB(int arg_cnt, char **args) {
	flagShowSignalB = true;
}

void cmd_showADCSignalL(int arg_cnt, char **args) {
	flagShowADCSignalL = true;
}

void cmd_showADCSignalR(int arg_cnt, char **args) {
	flagShowADCSignalR = true;
}

void cmd_showADCSignalB(int arg_cnt, char **args) {
	flagShowADCSignalB = true;
}

void cmd_showADCSignalLRaw(int arg_cnt, char **args) {
	flagShowADCSignalLRaw = true;
}

void cmd_showADCSignalRRaw(int arg_cnt, char **args) {
	flagShowADCSignalRRaw = true;
}

void cmd_showADCSignalBRaw(int arg_cnt, char **args) {
	flagShowADCSignalBRaw = true;
}

void cmd_showValuesSendToDue(int arg_cnt, char **args) {
	flag_ShowValuesSendToMaster = true;
}

void cmd_showOffset(int arg_cnt, char **args) {
	flagShowOffset = true;
}

void cmd_showLoopTime(int arg_cnt, char **args) {
	flagShowLoopTime = true;
}

void cmd_hideValues(int arg_cnt, char **args) {
	flagShowSignalL = false;
	flagShowSignalR = false;
	flagShowSignalB = false;
	flagShowADCSignalL = false;
	flagShowADCSignalR = false;
	flagShowADCSignalB = false;
	flagShowADCSignalLRaw = false;
	flagShowADCSignalRRaw = false;
	flagShowADCSignalBRaw = false;
	flag_ShowValuesSendToMaster = false;
	flagShowOffset = false;
	flagShowLoopTime = false;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
