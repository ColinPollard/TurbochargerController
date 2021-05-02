/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void transmitString(char* input);

// *** Global Variables ***
// Buffer to hold the ascii representation of the desired valve positions.
volatile char numberBuffer[3];
// Current state of receive interrupt.
volatile int readState = 0;
// Current index of buffer
volatile int bufferIndex = 0;
// Desired stepper motor position in steps.
volatile int desiredStep = 0;
// Current stepper motor position in steps.
int currentStep = 0;
// Desired solenoid duty cycle from 0-333
volatile int solenoidDuty = 0;

/*
 * Handles receiving from RS232. Sets desired positions.
 */
void USART2_IRQHandler(void)
{
	char readCharacter = USART2->RDR;
	
	// State 0 means there has been no prior transmission
	if (readState == 0)
	{
		// Desired motor command sent
		if (readCharacter == 'M')
			readState = 1;
		// Desired solenoid command sent
		else if (readCharacter == 'S')
			readState = 2;
	}
	
	// Motor position incoming.
	else if (readState == 1)
	{
		// End of message
		if (readCharacter != '\n')
		{
			// Buffer is not full
			if (bufferIndex < 3)
			{
				numberBuffer[bufferIndex] = readCharacter;
				bufferIndex++;
			}
			// Buffer is full, error state.
			else
			{
				readState = 0;
				bufferIndex = 0;
			}
		}
		
	}
	
	else if (readState == 2)
	{
		
	}
}


/*
 * Initializes ADC to PA2
 */
void ADC_init(void) {
  //Use PC0 as ADC12_IN12
  //Configure to Analog Mode (MODER bits 0 and 1 to 1)
  GPIOA->MODER |= 0x3 << 14;

  //Set no pull-up/pull-down resistors (PUPDR bits 0 and 1 to 0)
  GPIOA->PUPDR &= ~(0x3 << 14);

  //Enable ADC1 in RCC Peripheral
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

  //Configure ADC1 to 12-bit resolution, continuous conversion, hardware triggers disabled
  ADC1->CFGR |= (1<<13);
  ADC1->CFGR &= ~((1<<3)|(1<<4)|(1<<5)|(1<<10)|(1<<11));
	
  //Enable Channel 12 in the ADC
  //ADC1->CHSELR |= 1 << 12;

  //Calibrate the ADC
  //*****Clear ADEN
  if ((ADC1->CR & ADC_CR_ADEN) != 0) {
    ADC1->CR |= ADC_CR_ADDIS;
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0){} 

  //*****Clear DMAEN
  ADC1->CFGR &= ~ADC_CFGR_DMAEN;

  //*****Start Calibratiom
  ADC1->CR |= ADC_CR_ADCAL;

  //*****Wait until calibration is complete
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) {}
  
  //Enable/start the ADC
  //*****Clear ADRDY
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
    ADC1->ISR |= ADC_ISR_ADRDY;
  }

  //*****Enable the ADC
  ADC1->CR |= ADC_CR_ADEN;

  //*****Wait until the ADC is ready
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}

  //*****Start the ADC
  ADC1->CR |= ADC_CR_ADSTART;
}

/*                                                                                                                                                                                                         
 * Converts integer to asci byte array                                                                                                                                                                     
 */
char* int_to_string(uint16_t num) {
  static char bytes[] = {'0','0','0','0','\n','\0'};

  int i = 0;
  while(num != 0) {
    int dig = num % 10;
    bytes[3 - i] = dig + 48;
    i++;
    num /= 10;
  }
  return bytes;
}

/*
 * Reads value of ADC and transmits through UART
 */
void check_ADC(void) {
    //Read ADC Data
    uint16_t data = ADC1->DR;// & 0xFFF;
    char* value = int_to_string(data);
    
    //Transmit the value
    transmitString(value);
}

// Character transmitting function
void transmitChar(char input)
{
	// Check and wait for the USART transmit status flag. (Transmit data register empty TXE)
	while(!(USART2->ISR & (1 << 7)))
	{
		// Do Nothing
	}
	// Write character into transmit data register
	USART2->TDR = input;
}

// Transmit an entire string
void transmitString(char* input)
{
	int index = 0;
	for(; input[index] != 0; index++)
	{
		transmitChar(input[index]);
	}
}

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
  // MX_USART2_UART_Init();
  // MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	
	// Initialize the USART3 module TX is PA2, RX is PA3
	// Set moder to alternate function 10. Clear bit, then set bit.
	GPIOA->MODER &= ~((1 << 4) | (1 << 6));
	GPIOA->MODER |= ((1 << 5) | (1 << 7));
	// Set the alternate functions of each pin. Alternate Function 7 (0111) on both.
	GPIOA->AFR[0] &= ~((1 << 11) | (1 << 15));
	// Set the last bits next
	GPIOA->AFR[0] |= ((7 << 8) | (7 << 12));
	
	// Route clock
	RCC->APB1ENR1 |= (1 << 17);
	// RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	// Set baud rate 115200
	USART2->BRR = (HAL_RCC_GetHCLKFreq() / 115200);
	// Enable the transmitter and receiver
	USART2->CR1 |= ((1 << 2) | (1 << 3));
	// Enable receive not empty interrupt
	USART2->CR1 |= (1 << 5);
	// Enable the peripheral
	USART2->CR1 |= (1 << 0);
	
	// Configure the NVIC
	// Enable the interrupt
	NVIC_EnableIRQ(USART2_IRQn);
	// Set the priority of the interrupt
	NVIC_SetPriority(USART2_IRQn, 1);
	
	// Enable the stepper motor
	GPIOB->ODR &= ~(1 << 0);
	// Set the direction
  GPIOB->ODR |= (1 << 6);
	// Initialize ADC
	//ADC_init();
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// Step the motor.
		// GPIOB->ODR ^= (1 << 7);
		transmitString("Hello");
		//check_ADC();
		
		GPIOB->ODR ^= (1 << 3);
		HAL_Delay(10);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(Solenoid_Control_GPIO_Port, Solenoid_Control_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Step_Enable_Pin|LED_Pin|Step_Dir_Pin|Step_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Solenoid_Control_Pin */
  GPIO_InitStruct.Pin = Solenoid_Control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Solenoid_Control_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Step_Enable_Pin LED_Pin Step_Dir_Pin Step_Step_Pin */
  GPIO_InitStruct.Pin = Step_Enable_Pin|LED_Pin|Step_Dir_Pin|Step_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
