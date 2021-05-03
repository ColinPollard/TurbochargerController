/**
  ******************************************************************************
  * File           : main.c
  * Project        : ECE 5780 -- MiniProject
  * Authors        : Colin Pollard, McKay Mower, Luke Majors
  * Date           : May 3, 2021
  * Description    : Turbocharger controller main code to control STM32L412 microcontroller.
  *                  Reads data from pressure sensor using ADC, transmits pressure and recieves
  *                  commands from external controller using UART protocol, and controls stepper
  *                  motor using this information.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Forward Declarations ------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void ADC_init(void);
void PWM_init(void);
void transmitChar(char input);
void transmitString(char* input);
volatile char* int_to_string(uint16_t num);
uint16_t string_to_int(volatile char* str);

/* Global Variables --------------------------------------------------------*/
volatile char numberBuffer[3];                        // Buffer to hold the ascii representation of the desired valve positions.
volatile int readState = 0;                           // Current state of receive interrupt.
volatile int bufferIndex = 0;                         // Current index of buffer
volatile uint16_t desiredPressure = 1;                // Desired stepper motor position in steps.
int currentStep = 1;                                  // Current stepper motor position in steps.
volatile int solenoidDuty = 0;                        // Desired solenoid duty cycle from 0-333
volatile uint16_t actualPressure = 0;                 // Current pressure
volatile char bytes[] = {'0','0','0','0','\n','\0'};  // Byte array to help convert integers to ascii 

/**
 * Initializes and calibrates ADC1 channel 12 to pin PA7
 */
void ADC_init(void) {
  // Configure PA7 to Analog Mode (MODER bits 14 and 15 to 1)
  GPIOA->MODER |= 0x3 << 14;

  // Set PA7 to no pull-up/pull-down resistors (PUPDR bits 14 and 15 to 0)
  GPIOA->PUPDR &= ~(0x3 << 14);

  // Enable ADC1 in RCC Peripheral
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

  // Ensure DEEPPWD is set to 0
  if((ADC1 -> CR & ADC_CR_DEEPPWD))
    ADC1->CR &= ~ADC_CR_DEEPPWD;

  // Set ADVREGEN to enable voltage regualtor start up
  ADC1->CR |= ADC_CR_ADVREGEN;
  HAL_Delay(1); // Wait for voltage regulator

  // Configure ADC1 to 12-bit resolution, single conversion, hardware triggers disabled
  ADC1->CFGR &= ~((1<<3)|(1<<4)|(1<<5)|(1<<10)|(1<<11)|(1<<16)|(1<<14)|(1<<31)|(1<<25)|(1<<24)|(1<<23)|(1<<13));

  // Set Channel 12 to single-ended input conversion
  ADC1->DIFSEL &= ~(1 << 12);

	// Set ADC1 to perform single-ended input conversion
  ADC1->CR &= ~(1 << 30);
	
  // Calibrate the ADC
  //**** Clear ADEN
  if ((ADC1->CR & ADC_CR_ADEN) != 0)
    ADC1->CR |= ADC_CR_ADDIS;

  while ((ADC1->CR & ADC_CR_ADEN) != 0){}

  //**** Start Calibratiom
  ADC1->CR |= ADC_CR_ADCAL;
 
  //**** Wait until calibration is complete
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) {}
 
  // Enable/start the ADC
  //**** Clear ADRDY
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
    ADC1->ISR |= ADC_ISR_ADRDY;
  }

  //**** Enable the ADC
  ADC1->CR |= ADC_CR_ADEN;

  //**** Wait until the ADC is ready
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}

  // Select channel 12 and select one conversion
  ADC1->SQR1 &= ~(0xF | (0x1F << 6));
  ADC1->SQR1 |= (12 << 6);
  
  // Start the ADC
  ADC1->CR |= ADC_CR_ADSTART;
}

/**                                                                                                                                                                                                      
 * Converts an integer to an ascii representation for UART transmission
 * The converted character array include leading zeros for numbers below
 * four digits, ends in a newline character and is null terminated.
 * 
 * @param    num -- the integer to convert
 * @return   A character array representing the integer                                                                                                                                                      
 */
volatile char* int_to_string(uint16_t num) {
  // Reinitialize the byte array
  for(int i = 0; i < 4; i++) {
		bytes[i] = '0';
	}

  // Convert each digit into an ascii character and add to the array
  int i = 0;
  while(num != 0) {
    int dig = num % 10;
    bytes[3 - i] = dig + 48;
    i++;
    num /= 10;
  }
  return bytes;
}

/**
 * Converts an array of character representing an integer 
 * into and integer type.
 * The input character array is assumed to represent a 3 digit number,
 * or a smaller number that includes leading zeros.
 * 
 * @param   str -- A string or character array representing an integer
 * @return  An integer value associated with the input character array
 */
uint16_t string_to_int(volatile char* str) {
  char t;
  uint16_t num = 0;

  for(int i = 0; i < 3; i++) {
    t = str[i];
    uint16_t dig = t - 48;
    num *= 10;
    num += dig;
  }
  return num;
}

/**
 * Transmits a character using the UART1 peripheral
 * 
 * @param   input -- the character to transmit
 * @return  None
 */
void transmitChar(char input)
{
	// Check and wait for the USART transmit status flag. (Transmit data register empty TXE)
	while(!(USART1->ISR & (1 << 7)))
	{
		// Do Nothing
	}
	// Write character into transmit data register
	USART1->TDR = input;
}

/**
 * Transmits a string using the UART1 peripheral
 * 
 * @param   input -- the string to transmit
 * @return  None
 */
void transmitString(char* input)
{
	int index = 0;
	for(; input[index] != 0; index++)
	{
		transmitChar(input[index]);
	}
}

/**
 * Interrupt handler for USART 1
 * Parses received data from RS232 
 * Sets desired motor positions and solenoid PWM duty cycle accordingly
 * @param   input -- the string to transmit
 * @return  None
 */
void USART1_IRQHandler(void)
{
	char readCharacter = USART1->RDR;

  // Echo the character back
	transmitChar(readCharacter);
	
	// State 0 means there has been no prior transmission
	if (readState == 0)
	{
		// transmitString("Initial\r\n");
		// Desired motor command sent
		if (readCharacter == 'P')
			readState = 1;
		// Desired solenoid command sent
		else if (readCharacter == 'S')
			readState = 2;
	}
	
	// Numbers Incoming.
	else if (readState == 1 || readState == 2)
	{
		// Not end of line
		if (readCharacter != '\n')
		{
			// Buffer is not full
			if (bufferIndex < 3)
			{
				// Check that number is in range
				if (readCharacter >= '0' && readCharacter <= '9')
				{
					numberBuffer[bufferIndex] = readCharacter;
					bufferIndex++;
				}
				// Invalid character received, error state.
				else 
				{
					readState = 0;
					bufferIndex = 0;
				}
			}
			// Buffer is full, error state.
			else
			{
				readState = 0;
				bufferIndex = 0;
			}
		}
		// End line received
		else
		{
			// Check that the buffer is full
			if (bufferIndex == 3)
			{
				// Convert the string to an integer
				uint16_t temp = string_to_int(numberBuffer);
				
				// Motor command was processed
				if (readState == 1)
					desiredPressure = temp;
					
				else if (readState == 2)
					TIM2->CCR1 = temp;
				
				bufferIndex = 0;
				readState = 0;
			}
			// Buffer not full, error state
			else
			{
				readState = 0;
				bufferIndex = 0;
			}
		}
	}
}

/**
 * Initializes the PWM output on pin pA5 using the TIM2 peripheral
 */
void PWM_init(void){
	__HAL_RCC_TIM2_CLK_ENABLE();
	TIM2->PSC = 2999;
	TIM2->ARR = 333;
	
	// OC2M CONFIGURE TO PCM MODE 1
	TIM2->CCMR1 &= ~(1 << 6 | 1 << 5 | 1 << 4);
	TIM2->CCMR1 |= (1 << 6 | 1 << 5);
	
	// Enable ccer output
	TIM2->CCER |= 1 << 0;
	
	// 50% duty cycle
	TIM2->CCR1 = 0;
	
	// Set pa5 to alt function
	GPIOA->MODER &= ~(1 << 11 | 1 << 10);
	GPIOA->MODER |= 1 << 11;
	
	// pa5 alt function 1 = TIM2_CH1
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5);
	GPIOA->AFR[0] |= 1 << 20; // choose alt function 0001
	
	
	// Enable counter last
	TIM2->CR1 |= (1 << 0);
}

/**
 * Initializes the USART1 peripheral
 */
void UART_Init(void) {
  // Don't Initialize the USART3 module TX is PA2, RX is PA3
	// Initialize the USART2 module TX is PA9, RX is PA10
	// Set moder to alternate function 10. Clear bit, then set bit.
	GPIOA->MODER &= ~((1 << 18) | (1 << 20));
	GPIOA->MODER |= ((1 << 19) | (1 << 21));
	// Set the alternate functions of each pin. Alternate Function 7 (0111) on both.
	GPIOA->AFR[1] &= ~((1 << 7) | (1 << 11));
	// Set the last bits next
	GPIOA->AFR[1] |= ((7 << 4) | (7 << 8));
	
	// Route clock
	__HAL_RCC_USART1_CLK_ENABLE();

	// Set baud rate 115200
	USART1->BRR = (HAL_RCC_GetHCLKFreq() / 115200);
	// Enable the transmitter and receiver
	USART1->CR1 |= ((1 << 2) | (1 << 3));
	// Enable receive not empty interrupt
	USART1->CR1 |= (1 << 5);
	// Enable the peripheral
	USART1->CR1 |= (1 << 0);
	
	// Configure the NVIC for USART1
	// Enable the interrupt
	NVIC_EnableIRQ(USART1_IRQn);
	// Set the priority of the interrupt
	NVIC_SetPriority(USART1_IRQn, 1);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	ADC_init();
	PWM_init();
  UART_init();
	
	// Enable the stepper motor
	GPIOB->ODR &= ~(1 << 0);
	// Set the direction
  GPIOB->ODR |= (1 << 6);

  /* Infinite loop */
  while (1)
  {
    //Transmit actual and desired pressure readings
		transmitString("Current Pressure Reading: ");
		volatile char* value = int_to_string(actualPressure);
    transmitString((char*)value);
		transmitString("Desired Pressure Reading: ");
		transmitString(int_to_string(desiredPressure));
		transmitChar('\n');

		HAL_Delay(1000);
  }
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
  //HAL_GPIO_WritePin(Solenoid_Control_GPIO_Port, Solenoid_Control_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Step_Enable_Pin|LED_Pin|Step_Dir_Pin|Step_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Solenoid_Control_Pin */
	/*
  GPIO_InitStruct.Pin = Solenoid_Control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Solenoid_Control_GPIO_Port, &GPIO_InitStruct);
	*/
  /*Configure GPIO pins : Step_Enable_Pin LED_Pin Step_Dir_Pin Step_Step_Pin */
  GPIO_InitStruct.Pin = Step_Enable_Pin|LED_Pin|Step_Dir_Pin|Step_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

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
