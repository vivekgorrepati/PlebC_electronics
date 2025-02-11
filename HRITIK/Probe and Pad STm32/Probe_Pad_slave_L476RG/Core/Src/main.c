/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/*
 * USART - Slave
 * PA9  ->	USART1_Tx
 * PA10 ->	USART1_Rx
 * PA8	->	DE_RE_ENB
 *
 * For Analog connections
 * PC0	->	ADC1_IN1	->	JOYSTICK_X1
 * PC1	->	ADC1_IN2	->	JOYSTICK_Y1
 * PC2	->	ADC1_IN3	->	JOYSTICK_X2
 * PC3	->	ADC1_IN4	->	JOYSTICK_Y2
 * PA4	->	ADC1_IN9	->	POTENTIOMETER
 *
 * For debugging and Serial monitor printing
 * PA2	->	USART2_Tx
 * PA3	->	USART2_Rx
 */

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define FRAME_SIZE 13  // Expected frame length
#define NUM_CHANNELS 5  // Number of ADC channels
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t uartByte;              // Temporary byte buffer
uint32_t byteCount = 0;        // Counter to count bytes between 0xAA and 0x55
uint8_t isReceiving = 0;       // Flag to indicate whether we are receiving valid data
uint8_t imuBuffer[FRAME_SIZE]; // Buffer to store the frame (if needed)
uint8_t frameIndex = 0;        // Index for the current byte in the buffer
char uart_buffer[100];         // Buffer to hold the formatted string for transmission

uint8_t imuData[13];			// Buffer to copy received imu data
uint8_t loadcellData[7];		// Buffer to copy received loadcell data

// Declare variables to store quaternion components
float w;
float x;
float y;
float z;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t JoystickX1 = 0;
uint16_t JoystickY1 = 0;
uint16_t Pot = 0;
uint16_t JoystickX2 = 0;
uint16_t JoystickY2 = 0;

//uint16_t rawValues[5];
char uartBuffer[100];  // Buffer to send UART data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void processModbusFrame(void);
void processReceivedData(uint8_t *frame, uint8_t frameLength);
uint16_t calculateCRC(uint8_t *buffer, uint16_t length);	// CRC calculation function to validate data integrity

// Function to start UART receive process
//void UART_StartReceive(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Redirects printf output to the ITM (for debugging)
 * @param file: the file handle (unused)
 * @param ptr: pointer to the string to write
 * @param len: length of the string to write
 * @return number of characters written
 */
int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++)
        ITM_SendChar((*ptr++));		// Sends each character to ITM for debugging

    return len;
}


/**
 * @brief Calculates the CRC value for data integrity verification
 * @param data: pointer to the data buffer
 * @param length: length of the data
 * @return computed CRC value
 */

uint16_t calculateCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF; // Initial CRC value

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // XOR byte into the least significant byte of CRC

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001) // If the LSB is set
            {
                crc >>= 1;       // Shift right
                crc ^= 0xA001;   // Apply the polynomial
            }
            else
            {
                crc >>= 1; // Just shift right
            }
        }
    }

    return crc; // Return the CRC value
}

void processImuData()
{
	// Extract Quaternion Data
		int16_t modbusRegisters[4];
		for (int i = 0; i < 4; i++)
		{
			modbusRegisters[i] = (int16_t)((imuData[2 + (i * 2)] << 8) | imuData[3 + (i * 2)]);
		}

	// Validate CRC
		uint16_t calculatedCRC = calculateCRC(imuData, 10);
		uint16_t receivedCRC = (imuData[10]) | (imuData[11] << 8);

	if (calculatedCRC != receivedCRC)
	{
		snprintf(uart_buffer, sizeof(uart_buffer), "CRC mismatch: Expected 0x%04X, Got 0x%04X\n", calculatedCRC, receivedCRC);
		HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
		return;
	}

	 // Convert Quaternion Data to Floats
		float w = modbusRegisters[0] / 30000.0f;
		float x = modbusRegisters[1] / 30000.0f;
		float y = modbusRegisters[2] / 30000.0f;
		float z = modbusRegisters[3] / 30000.0f;

	// Debug Output Quaternion Data
		snprintf(uart_buffer, sizeof(uart_buffer), "%.4f,%.4f,%.4f,%.4f\r\n", w, x, y, z);
		HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

void processLoadcellData()
{
	// Extract Load Cell Data
		int16_t loadCellValue = 0;
		loadCellValue = (loadcellData[2] << 8) | loadcellData[3];

	// Validate CRC
		uint16_t calculatedCRC = calculateCRC(loadcellData, 4);
		uint16_t receivedCRC = (loadcellData[4]) | (loadcellData[5] << 8);

		if (calculatedCRC != receivedCRC)
		{
			snprintf(uart_buffer, sizeof(uart_buffer), "CRC mismatch: Expected 0x%04X, Got 0x%04X\n", calculatedCRC, receivedCRC);
			HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
			return;
		}

	// Debug Output Load Cell Data
		snprintf(uart_buffer, sizeof(uart_buffer), "Received Load Cell Data: %d\n", loadCellValue);
		HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

/**
 * @brief UART receive callback function (interrupt-based)
 * @param huart: pointer to UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // Check if the interrupt is from UART1
    {
        if (!isReceiving)
        {
            // Looking for start byte 0xAA
            if (uartByte == 0xAA)
            {
                isReceiving = 1;  // Start storing the frame
                frameIndex = 0;   // Reset buffer index
                imuBuffer[frameIndex++] = uartByte;  // Store 0xAA as the start byte
                byteCount = 1;
            }
        }
        else
        {
            // Store the incoming byte
            imuBuffer[frameIndex++] = uartByte;
            byteCount++;

            // If we received the end byte 0x55 and the frame length is correct
            if (uartByte == 0x55) 		//	if (uartByte == 0x55 && frameIndex == FRAME_SIZE)
            {

                if(byteCount == 13 && imuBuffer[1] == 0x01)
                {
                	memcpy(imuData, imuBuffer,sizeof(imuBuffer));
                }

                if(byteCount == 7 && imuBuffer[1] == 0x02)
				{
                	memcpy(loadcellData, imuBuffer, 7);
				}

                memset(imuBuffer, 0, sizeof(imuBuffer)); // Reset the entire buffer to 0

                // Reset receiving state for the next frame
                isReceiving = 0;
                frameIndex = 0;
                byteCount = 0;


            }
            // If we exceed the frame size without receiving 0x55, reset
            else if (frameIndex >= FRAME_SIZE)
            {
                isReceiving = 0;  // Reset receiving flag
                frameIndex = 0;   // Reset index
            }
        }
+
        // Restart UART reception for the next byte
        HAL_UART_Receive_IT(&huart1, &uartByte, 1);
    }
}


ADC_ChannelConfTypeDef sConfig;
ADC_ChannelConfTypeDef sConfig = {0};
ADC_MultiModeTypeDef multimode = {0};

void Read_ADC_Channel(uint32_t channel, uint16_t *result) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;  // Use the appropriate sampling time
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
		*result = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
}

void processRemoteData(void)
{
	  Read_ADC_Channel(ADC_CHANNEL_1, &JoystickX1);
	  Read_ADC_Channel(ADC_CHANNEL_2, &JoystickY1);
	  Read_ADC_Channel(ADC_CHANNEL_3, &Pot);
	  Read_ADC_Channel(ADC_CHANNEL_4, &JoystickX2);
	  Read_ADC_Channel(ADC_CHANNEL_9, &JoystickY2);

	  // Format sensor values into a string
	  snprintf(uartBuffer, sizeof(uartBuffer),"X1: %d, Y1: %d, X2: %d, Y2: %d, P: %d\r\n",JoystickX1, JoystickY1, Pot, JoystickX2, JoystickY2);

	  // Send the formatted string over UART1
	  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &uartByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
	   processImuData();
	   HAL_Delay(100);
	   processLoadcellData();
	   HAL_Delay(100);
	   processRemoteData();
	   HAL_Delay(100);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DE_RE_ENB_Pin */
  GPIO_InitStruct.Pin = DE_RE_ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_RE_ENB_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
