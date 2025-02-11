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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*
 * UART1 - Master
 * PA9 -> USART1_Tx
 * PB7 -> USART1_Rx
 *
 * PB3  -> GPIO_OUTPUT(CLK)
 * PA10 -> GPIO_INPUT(Data)
 *
 * PA2 -> USART2_Tx
 * PA3 -> USART2_Rx
 *
 * PB9 -> I2C1_SDA
 * PB8 -> I2C1_SCL
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include<bno055.h>
#include<loadcell.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t imuModbusFrame[12];				// MODBUS Frame // Frame: 1-byte address + 1-byte function + 2-byte start address + 2-byte quantity + 1-byte byte count + data + 2-byte CRC
int16_t imuQuatValues[4]; 				// Buffer to store Quat values // One 16-bit register for each scaled quaternion value

int16_t loadCellValue;					// store loadcell value
int16_t loadCellModbusRegister = 0;		// variable to store loadcell values for SendLoadCellOnModbus()

char uartBuffer[50];       				// Buffer to hold the string

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId IMUTaskHandle;
osThreadId LoadCellTaskHandle;
osThreadId SendingTaskHandle;
/* USER CODE BEGIN PV */

// Prepare the data for UART transmission


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartIMUTask(void const * argument);
void StartLoadCellTask(void const * argument);
void StartSendingTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++)
        ITM_SendChar((*ptr++));

    return len;
}

// Function to calculate Modbus CRC
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

void QuaternionToModbus(float w, float x, float y, float z)
{
    // Scale the float values to fit into the range of int16_t and store them
    imuQuatValues[0] = (int16_t)(w * 30000); // w component
    imuQuatValues[1] = (int16_t)(x * 30000); // x component
    imuQuatValues[2] = (int16_t)(y * 30000); // y component
    imuQuatValues[3] = (int16_t)(z * 30000); // z component
}

void SendImuDataOnModbus()
{
    uint8_t startByte = 0xAA;       	// Start byte for the frame
    uint8_t typeIdentifier = 0x01; 		// Identifier for Quaternion Data
    uint16_t crc;

    // Define frame buffer
    uint8_t imuModbusFrame[13]; // Start Byte (1) + Type ID (1) + Data (8) + CRC (2) + End (1)

    // Construct the custom frame
    imuModbusFrame[0] = startByte;         // Start Byte
    imuModbusFrame[1] = typeIdentifier;    // Type Identifier

    // Add quaternion data to the frame
    for (int i = 0; i < 4; i++) {
        imuModbusFrame[2 + (i * 2)] = (imuQuatValues[i] >> 8) & 0xFF; // High byte
        imuModbusFrame[3 + (i * 2)] = imuQuatValues[i] & 0xFF;        // Low byte
    }

    // Calculate CRC for the first 10 bytes (Start Byte + Type ID + Quaternion Data)
    crc = calculateCRC(imuModbusFrame, 10);
    imuModbusFrame[10] = crc & 0xFF;        // CRC Low Byte
    imuModbusFrame[11] = (crc >> 8) & 0xFF; // CRC High Byte

    // Add End Byte
    imuModbusFrame[12] = 0x55; // End of frame

    // Send the frame via UART
    if (HAL_UART_Transmit(&huart1, imuModbusFrame, 13, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle transmission error

    }
}

/* 	************************************************************************************************************ */

void LoadCellDataToModbus()
{
    // Convert the load cell data to int16_t

	loadCellValue = weigh();
	loadCellModbusRegister = loadCellValue;
	sprintf(uartBuffer,  "Load Cell Value: %d\r\n", loadCellValue);

	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

}

void SendLoadCellDataOnModbus()
{
    uint8_t startByte = 0xAA;       		// Start byte for the frame
    uint8_t typeIdentifier = 0x02;  		// Identifier for loadcell Data
    uint16_t crc;

    // Define frame buffer
    uint8_t loadCellModbusFrame[7]; 		// Start Byte (1) + Type ID (1) + Data (2) + CRC (2) + End (1)

    // Construct the custom frame
    loadCellModbusFrame[0] = startByte;        // Start Byte
    loadCellModbusFrame[1] = typeIdentifier;   // Type Identifier

    // Add load cell data (2 bytes)
    loadCellModbusFrame[2] = (loadCellModbusRegister >> 8) & 0xFF; 		// High byte
    loadCellModbusFrame[3] = loadCellModbusRegister & 0xFF;        		// Low byte

    // Calculate CRC for the first 4 bytes (Start Byte + Type ID + Load Cell Data)
    crc = calculateCRC(loadCellModbusFrame, 4);
    loadCellModbusFrame[4] = crc & 0xFF;        // CRC Low Byte
    loadCellModbusFrame[5] = (crc >> 8) & 0xFF; // CRC High Byte

    // Add End Byte
    loadCellModbusFrame[6] = 0x55; // End of frame

    // Send the frame via UART
    if (HAL_UART_Transmit(&huart1, loadCellModbusFrame, 7, HAL_MAX_DELAY) != HAL_OK) {
        // Handle transmission error

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    // Load Cell
	HAL_TIM_Base_Start(&htim1);
	setTare();

    // BNO055
	char init_imu_uart_buffer[100];

    // Initialize BNO055
	if (BNO055_Init() != HAL_OK)
	{
	  snprintf(init_imu_uart_buffer, sizeof(init_imu_uart_buffer),"BNO055 initialization failed\n");
	  HAL_UART_Transmit(&huart2, (uint8_t *)init_imu_uart_buffer, strlen(init_imu_uart_buffer), HAL_MAX_DELAY);
	  //return -1;
	}
	snprintf(init_imu_uart_buffer, sizeof(init_imu_uart_buffer),"BNO055 initialized successfully\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)init_imu_uart_buffer, strlen(init_imu_uart_buffer), HAL_MAX_DELAY);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 256);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of LoadCellTask */
  osThreadDef(LoadCellTask, StartLoadCellTask, osPriorityAboveNormal, 0, 256);
  LoadCellTaskHandle = osThreadCreate(osThread(LoadCellTask), NULL);

  /* definition and creation of SendingTask */
  osThreadDef(SendingTask, StartSendingTask, osPriorityLow, 0, 128);
  SendingTaskHandle = osThreadCreate(osThread(SendingTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartIMUTask */
/**
  * @brief  Function implementing the IMUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	Quaternion quat;
	char uart_buffer[100];
  /* Infinite loop */
  for(;;)
  {
	  if (BNO055_ReadQuaternion(&quat) == HAL_OK)
	  {
		  snprintf(uart_buffer, sizeof(uart_buffer), "%.4f,%.4f,%.4f,%.4f\r\n", quat.w, quat.x, quat.y, quat.z);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
		  QuaternionToModbus(quat.w, quat.x, quat.y, quat.z);

	  }
    osDelay(50);
  }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLoadCellTask */
/**
* @brief Function implementing the LoadCellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoadCellTask */
void StartLoadCellTask(void const * argument)
{
  /* USER CODE BEGIN StartLoadCellTask */
  /* Infinite loop */
  for(;;)
  {
	LoadCellDataToModbus();
    osDelay(50);

  }
  /* USER CODE END StartLoadCellTask */
}

/* USER CODE BEGIN Header_StartSendingTask */
/**
* @brief Function implementing the SendingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendingTask */
void StartSendingTask(void const * argument)
{
  /* USER CODE BEGIN StartSendingTask */
  /* Infinite loop */
  for(;;)
  {
	  SendImuDataOnModbus();
	  osDelay(50);
	  SendLoadCellDataOnModbus();
	  osDelay(50);
  }
  /* USER CODE END StartSendingTask */
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
