/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EEPROM_ADDR 0b10100000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

// Ceaser_Cipher
uint8_t RxBuffer[2];
uint8_t TxBuffer[100];
int button;
char character;
int condition = 0;
char state;


// calculator
uint8_t receive[2];
char sign;
char first_number;
char second_number;
char equal;
int state_cal = 0;
int condition_cal = 0;
uint16_t Answer = 0;


// PID 24LC64
uint32_t Kp = 0x12;
uint8_t Kp_sep[4];
uint32_t Ki = 0x22;
uint8_t Ki_sep[4];
uint32_t Kd = 0x13;
uint8_t Kd_sep[4];
uint32_t Kff = 0x44;
uint8_t Kff_sep[4];
uint8_t eepromWriteF = 0;
uint8_t eepromReadF = 0;
uint8_t eepromReadBack[16];
uint8_t PID_data[16];


// SPI_LED_5
uint8_t SPIRx[10];
uint8_t SPITx[10];
uint8_t LEDstate = 0;


// SPI_VOLT_ADC_6
uint16_t VOLTRx[16];
uint16_t VOLTTx[16];
//uint8_t LEDstate = 0;
uint16_t adcRawData[30];
float Voltage = 0;
float sum_Voltage = 0;
float avg_Voltage = 0;




uint32_t timestamp = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void EEPROMWrite();
void EEPROMRead(uint8_t *Rdata,uint16_t len);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Ceaser_Cipher
//  uint8_t text[] = "1: Encrypt \r\n2 : Decode\r\n";
//  HAL_UART_Transmit(&huart2, text, 25, 10);
//  UARTCeaserCipherConfig();


  // Calculator
//  UARTCalculatorConfig();
//	uint8_t text[] = "\r\n-------Calculator : press number please------\r\n";
//	HAL_UART_Transmit(&huart2, text, 50, 10);


  // SPI_LED_5
// 	SPITxRx_Setup();
// 	setOutputGPIOB();


  // SPI_VOLT_6
//  SPI_voltage_set();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // I2C_PID 24LC64
//	  Kp_Seperate(Kp);
//	  Ki_Seperate(Ki);
//	  Kd_Seperate(Kd);
//	  Kff_Seperate(Kff);
//	  EEPROMWrite();
//	  EEPROMRead(eepromReadBack,16);

	  // SPI_LED_5
//	  Set_SPI_LED();



	  // SPI_VOLT_6
//	  write_spi();
//
//	  static uint32_t timestamp = 0;
//	  if(HAL_GetTick() > timestamp)
//	  {
//		timestamp = HAL_GetTick() + 10;
//		LEDstate++;
//		LEDstate = LEDstate % 2;
//		switch (LEDstate)
//			{
//			case 0:
//				SPI_voltage_set();
//				break;
//			case 1:
	  MCP4922_SetVoltage(3300);
	  voltage();
//				write_voltage();
//				voltage();
//				break;
//		}
//	  }
//	  SPITxRx_readIO();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//ceaser_cipher

	// confic
//void UARTCeaserCipherConfig()
//{
//	//start UART in DMA Mode
//	HAL_UART_Receive_DMA(&huart2, RxBuffer, 1);
//}
//
//	 //check
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart2)
//	{
//		RxBuffer[1] = '\0';
//
//		if(RxBuffer[0]=='1')button = '1';
//		else if(RxBuffer[0]=='2')button = '2';
//		else if(RxBuffer[0]=='3')button = '3';
//		else if((97 <= RxBuffer[0]) && (RxBuffer[0]<= 122))button = 'z';
//		else if((65 <= RxBuffer[0]) && (RxBuffer[0]<= 90))button = 'z';
//		else button = '4';
//
//		ceaser_cipher(button);
//
//	}
//}

	 //switch case
//void ceaser_cipher(state)
//{
//	switch(state)
//		{
//			case '1':
//			{
//				if (condition == 0)
//				{
//				sprintf((char*)TxBuffer, "----ENCRYPT----\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 2;
//				}
//
//			}
//			break;
//
//			case '2':
//			{
//				if (condition == 0)
//				{
//				character = RxBuffer[0]-3;
//				sprintf((char*)TxBuffer, "----DECODE----\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 3;
//				}
//			}
//			break;
//
//			case '3':
//			{
//				if ((condition == 0) || (condition == 1) || (condition == 2) || (condition == 3))
//				{
//				character = RxBuffer[0]+3;
//				sprintf((char*)TxBuffer, "1: Encrypt \r\n2: Decode\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 0;
//				}
//			}
//			break;
//
//			case '4':
//			{
//				if (condition == 0)
//				{
//				character = RxBuffer[0]+3;
//				sprintf((char*)TxBuffer, "again\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 0;
//				}
//				else if (condition == 1)
//				{
//				character = RxBuffer[0]+3;
//				sprintf((char*)TxBuffer, "again\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 1;
//				}
//				else if (condition == 2)
//				{
//				character = RxBuffer[0]+3;
//				sprintf((char*)TxBuffer, "again\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 2;
//				}
//				else if (condition == 3)
//				{
//				character = RxBuffer[0]+3;
//				sprintf((char*)TxBuffer, "again\r\n");
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 3;
//				}
//			}
//			break;
//
//			case 'z':
//			{
//				if ((condition == 2) || (condition == 1))
//				{
//				character = RxBuffer[0]+3;
//					if ((97 <= RxBuffer[0]) && (RxBuffer[0]<= 122) && (character > 122))
//					{
//						character = (character%122) + 96;
//					}
//					else if ((65 <= RxBuffer[0]) && (RxBuffer[0]<= 90) && (character > 90))
//					{
//						character = (character%122) + 64;
//					}
//				sprintf((char*)TxBuffer, "Received : %c\r\n3 : back\r\n",character);
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 2;
//				}
//				else if ((condition == 3) || (condition == 1))
//				{
//				character = RxBuffer[0]-3;
//					if ((97 <= RxBuffer[0]) && (RxBuffer[0]<= 122) && (character < 97))
//					{
//						character = character + 26;
//					}
//					else if ((65 <= RxBuffer[0]) && (RxBuffer[0]<= 90) && (character < 65))
//					{
//						character = character + 26;
//					}
//				sprintf((char*)TxBuffer, "Received : %c\r\n3 : back\r\n",character);
//				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//				condition = 3;
//				}
//			}
//			break;
//		}
//
//}



// Calculator

	// config
//void UARTCalculatorConfig()
//{
//	//start UART in DMA Mode
//	HAL_UART_Receive_DMA(&huart2, receive, 1);
//}

	//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart2)
//	{
//		receive[1] = '\0';
//
//		if((48 <= receive[0]) && (receive[0]<= 59) && (condition_cal == 0))
//		{
//			first_number = receive[0]% 48;
//			state_cal = 0;
//			condition_cal = 0;
//		}
//		else if((48 <= receive[0]) && (receive[0]<= 59))
//		{
//			second_number = receive[0]% 48;
//			state_cal = 2;
//			if(condition_cal == 1)condition_cal = 1;
//			else if (condition_cal == 3)condition_cal = 3;
//		}
//		else if((receive[0]=='+') || (receive[0]=='-') || (receive[0]=='*') || (receive[0]=='/'))
//		{
//			sign = receive[0];
//			state_cal = 1;
//			if((condition_cal == 0) || (condition_cal == 1))condition_cal = 1;
//			else if (condition_cal == 3)condition_cal = 3;
//		}
//		else if (((receive[0] == 13) && (condition_cal == 1)) && ((state_cal == 2)||(state_cal == 5)))
//		{
//			equal = receive[0];
//			state_cal = 3;
//			condition_cal = 3;
//		}
//		else if((receive[0] == 13) && (condition_cal == 3))
//		{
//			equal = receive[0];
//			state_cal = 4;
//			condition_cal = 3;
//		}
//		else
//		{
//			state_cal = 5;
//		}
//
//		Calculator(state_cal);
//	}
//}
//
//void Calculator(state)
//{
//	switch(state)
//	{
//		case 0:
//		{
//			sprintf((char*)TxBuffer, "%d", first_number);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//
//		case 1:
//		{
//			sprintf((char*)TxBuffer, "%c", sign);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//
//		case 2:
//		{
//			sprintf((char*)TxBuffer, "%d", second_number);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//
//		case 3:
//		{
//			if(sign=='+')Answer = first_number + second_number;
//			else if(sign=='-')Answer = first_number - second_number;
//			else if(sign=='*')Answer = first_number * second_number;
//			else if((sign=='/') && (second_number==0))Answer = 0;
//			else if((sign=='/') && (second_number!=0))Answer = first_number / second_number;
//			sprintf((char*)TxBuffer, "=%d", Answer);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//
//		case 4:
//		{
//			if(sign=='+')Answer += second_number;
//			else if(sign=='-')Answer = Answer - second_number;
//			else if((sign=='/') && (second_number==0))Answer = 0;
//			else if((sign=='/') && (second_number!=0))Answer = Answer / second_number;
//			sprintf((char*)TxBuffer, "=%d", Answer);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//
//		case 5:
//		{
//			sprintf((char*)TxBuffer, "\r\nplease press number again\r\n%d", Answer);
//			HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
//		}
//		break;
//	}
//
//}




// PID 24LC64

void Kp_Seperate(Kp)
{
	Kp_sep[0] = (uint8_t)(Kp >> 24) & 0xFF;
	Kp_sep[1] = (uint8_t)(Kp >> 16) & 0xFF;
	Kp_sep[2] = (uint8_t)(Kp >> 8) & 0xFF;
	Kp_sep[3] = (uint8_t)(Kp) & 0xFF;
}

void Ki_Seperate(Ki)
{
	Ki_sep[0] = (uint8_t)(Ki >> 24) & 0xFF;
	Ki_sep[1] = (uint8_t)(Ki >> 16) & 0xFF;
	Ki_sep[2] = (uint8_t)(Ki >> 8) & 0xFF;
	Ki_sep[3] = (uint8_t)(Ki) & 0xFF;
}

void Kd_Seperate(Kd)
{
	Kd_sep[0] = (uint8_t)(Kd >> 24) & 0xFF;
	Kd_sep[1] = (uint8_t)(Kd >> 16) & 0xFF;
	Kd_sep[2] = (uint8_t)(Kd >> 8) & 0xFF;
	Kd_sep[3] = (uint8_t)(Kd) & 0xFF;
}

void Kff_Seperate(Kff)
{
	Kff_sep[0] = (uint8_t)(Kff >> 24) & 0xFF;
	Kff_sep[1] = (uint8_t)(Kff >> 16) & 0xFF;
	Kff_sep[2] = (uint8_t)(Kff >> 8) & 0xFF;
	Kff_sep[3] = (uint8_t)(Kff) & 0xFF;
}


void EEPROMWrite()
{
    if (eepromWriteF && hi2c1.State == HAL_I2C_STATE_READY)
    {
        static uint8_t PID_data[16];

        PID_data[0] = Kp_sep[0];
        PID_data[1] = Kp_sep[1];
        PID_data[2] = Kp_sep[2];
        PID_data[3] = Kp_sep[3];
        PID_data[4] = Ki_sep[0];
        PID_data[5] = Ki_sep[1];
        PID_data[6] = Ki_sep[2];
        PID_data[7] = Ki_sep[3];
        PID_data[8] = Kd_sep[0];
        PID_data[9] = Kd_sep[1];
        PID_data[10] = Kd_sep[2];
        PID_data[11] = Kd_sep[3];
        PID_data[12] = Kff_sep[0];
        PID_data[13] = Kff_sep[1];
        PID_data[14] = Kff_sep[2];
        PID_data[15] = Kff_sep[3];

        if (HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_ADDR, 0x30, I2C_MEMADD_SIZE_16BIT, PID_data, 16) == HAL_OK)
        {
            eepromWriteF = 0; // Clear the write flag after initiating the write operation
        }
        else
        {
            // Error handling if write operation fails
        }
    }
}

void EEPROMRead(uint8_t *Rdata,uint16_t len)
{
	if (eepromReadF && hi2c1.State == HAL_I2C_STATE_READY)
	{
		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x30, I2C_MEMADD_SIZE_16BIT, Rdata, len);

		eepromReadF = 0;
	}
}



// I2C_LED_4

//void SPITxRx_readIO()
//{
//	SPITx[0] = 0b01000001;
//	SPITx[1] = 0x12;
//	SPITx[2] = 0;
//	SPITx[3] = 0;
//	HAL_I2C_Mem_Read_IT(hi2c, 0x20, 0x12, I2C_MEMADD_SIZE_16BIT, SwitchData, 8);
//	HAL_I2C_Master_Transmit_IT(&hi2c3, DevAddress, pData, Size)
//}

//void HAL_I2C_TxRxCpltCallback(I2C_HandleTypeDef *hspi)
//{
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //CS dnSelect
//}


// SPI_LED_5
//void SPITxRx_Setup()
//{
//	//CS pulse
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); // CS deSelect
//	HAL_Delay(1);
//}
//
//void SPITxRx_readIO()
//{
//	if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000001;
//		SPITx[1] = 0x12;
//		SPITx[2] = 0;
//		SPITx[3] = 0;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 4);
//	}
//}
//
//
//void setOutputGPIOB()
//{
//	if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x01;
//		SPITx[2] = 0b00000000;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//	}
//}
//
//void SPITxRx_writeIO()
//{
//	if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
//	{
//		if(SPIRx[2] == 0b00000010)
//		{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b10101010;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(500);
//
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b01010101;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(500);
//		}
//		else if(SPIRx[2] == 0b00000011)
//		{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b11111111;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		}
//		else if(SPIRx[2] == 0b00000001)
//		{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b00111111;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(100);
//
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b11001111;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(100);
//
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b11110011;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(100);
//
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
//		SPITx[0] = 0b01000000;
//		SPITx[1] = 0x13;
//		SPITx[2] = 0b11111100;
//		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
//		HAL_Delay(100);
//		}
//	}
//}
//
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //CS dnSelect
//}
//
//
//void Set_SPI_LED()
//{
//	static uint32_t timestamp = 0;
//	if(HAL_GetTick() > timestamp)
//	{
//		timestamp = HAL_GetTick() + 10;
//		LEDstate++;
//		LEDstate = LEDstate % 2;
//		switch (LEDstate)
//		{
//		case 0:
//		  SPITxRx_readIO();
//		  break;
//		case 1:
//		  SPITxRx_writeIO();
//		  break;
//		}
//	}
//}






	//SPI_VOLT_ADC_6
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcRawData, 30);
	}
}

void voltage()
{
	sum_Voltage = 0;
	for(int i=0;i<30;i++)
	{
		if(i%2 == 1)
		{
			Voltage = ((adcRawData[i]*3300)/4096);
			sum_Voltage = sum_Voltage + Voltage;
		}
		else if (i%2 == 0)
		{
			Voltage = ((adcRawData[i]*3300)/4096);
			sum_Voltage = sum_Voltage + Voltage;
		}
	}
	avg_Voltage = sum_Voltage/10;
}


void SPI_voltage_set()
{
    // CS pulse
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);  // CS Select
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);    // CS deSelect
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1);
}

void write_voltage()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

        //uint16_t VOLTTx[16] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1};
        uint16_t VOLTTx[16] = {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};


        HAL_SPI_Transmit_IT(&hspi2, (uint8_t*)VOLTTx, 16);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);  // CS deSelect
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void MCP4922_Select(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);  // Pull CS pin low
}

// Function to deselect the MCP4922 DAC
void MCP4922_Deselect(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);    // Pull CS pin high
}

uint16_t MCP4922_ConvertVoltage(float voltage)
{

  uint16_t digitalValue = (uint16_t)((voltage / 3300) * 4095);

  uint16_t dataWord = 0;
  dataWord |= (0 << 15);        // Leftmost bit is always 0
  dataWord |= (0 << 14);        // Buffered output
  dataWord |= (1 << 13);        // 1x gain
  dataWord |= (1 << 12);        // Power on DAC output
  dataWord |= (digitalValue & 0xFFF);  // 12-bit digital value

  return dataWord;
}

void MCP4922_SetVoltage(float voltage)
{
  uint16_t dataWord = MCP4922_ConvertVoltage(voltage);

  // Select MCP4922 DAC
  MCP4922_Select();

  // Transmit data word via SPI
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&dataWord, sizeof(dataWord), HAL_MAX_DELAY);

  // Deselect MCP4922 DAC
  MCP4922_Deselect();
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
