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
#include "PlebcStepper.h"
#include "modbus_rtu_slave.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//For Encoder
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

int prev_input_distance = 0;
int prevstepsToMove=0;

volatile float rpm = 0; // Current RPM
volatile float prev_rpm = 0;

float acceleration = 0;
float prev_acceleration = 0;

//Calculating motor steps
int motorSetSteps = 0; // Steps per revolution of the motor
float prev_targetPosition = 0;
int input_distance= 0;
long int updatedEncoderValue;
long int currentPosition = 0;
float targetPosition = 0;
long int positionToMove = 0;
int stepsToMove = 0;
//float enc_val_in_1rev = 1130.00; //linear encoder values in 1 revolution - test bench
//float mm_in_1rev = 40.00; //distance covered on 1 revolution (in mm)(linear encoder) - test bench

float enc_val_in_1rev = 2848.00; //linear encoder values in 1 revolution - x-axis
float mm_in_1rev = 100.00; //distance covered on 1 revolution (in mm)(linear encoder) - x-axis

//float enc_val_in_1rev = 3550.00; //linear encoder values in 1 revolution - z-axis
//float mm_in_1rev = 125.00; //distance covered on 1 revolution (in mm)(linear encoder) - z-axis



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Pins on Board */
//	PA0 -> STEP/Pulse
//	PA1 -> DIR
//  PA4 -> DRIVE_ENB
//	PB0 -> ENCODER_A
//	PB1 -> ENCODER_B
//	PC1 -> LIMIT_SW
//	PA9 -> USART1_TX
//	PA10 -> USART1_RX
//  PA8 -> DE_RE_Pin (DE,RE Pin of RS485 module)

/* User Instructions */

//	1mm = 160 steps
//  360 degree = 1130 linear encoder values
//  360 degree = 40 mm movement
//  1 mm = 28.25 linear encoder values
//  1 value = 0.353982300884956

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId EncoderTaskHandle;
osThreadId MotorTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartEncoderTask(void const * argument);
void StartMotorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateEncoder(void)
{
  int MSB = HAL_GPIO_ReadPin(GPIOB, ENCODER_A_Pin);
  int LSB = HAL_GPIO_ReadPin(GPIOB, ENCODER_B_Pin);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  lastEncoded = encoded;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ENCODER_A_Pin || GPIO_Pin == ENCODER_B_Pin)
  {
    updateEncoder();

  }
}

// Function to start UART reception
void UART_StartReceive(void)
{
  // Enable reception by setting DE and RE pins high
  HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_RESET);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, RX_BUFFER_SIZE);

}

int _write(int file, char *ptr, int len)
{
	int i = 0;
	for(i=0; i<len; i++)

		ITM_SendChar((*ptr++));

	return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	rpm = Holding_Registers_Database[0];
	acceleration = Holding_Registers_Database[1];
	motorSetSteps = Holding_Registers_Database[3];


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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  UART_StartReceive();

//  rpm = Holding_Registers_Database[0];
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
  /* definition and creation of EncoderTask */
  osThreadDef(EncoderTask, StartEncoderTask, osPriorityNormal, 0, 128);
  EncoderTaskHandle = osThreadCreate(osThread(EncoderTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityBelowNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 90;
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_Pin|DIR_Pin|DRIVE_ENB_Pin|DE_RE_ENB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIMIT_SW_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Pin DIR_Pin DRIVE_ENB_Pin DE_RE_ENB_Pin */
  GPIO_InitStruct.Pin = STEP_Pin|DIR_Pin|DRIVE_ENB_Pin|DE_RE_ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_A_Pin ENCODER_B_Pin */
  GPIO_InitStruct.Pin = ENCODER_A_Pin|ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEncoderTask */
/**
  * @brief  Function implementing the EncoderTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	// Define motor configurations
//	  MotorConfig motor1 = {GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_0};
//	int distance = Holding_Registers_Database[4];
//	int prev_distance = Holding_Registers_Database[4];
  /* Infinite loop */

  for(;;)
  {

	Input_Registers_Database[0] = encoderValue; // Store the encoder value in the first input register
	int distance_covered = encoderValue * (mm_in_1rev/enc_val_in_1rev);
	Input_Registers_Database[1] = distance_covered; // Store the encoder value in the first input register

    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/

// Function to initialize motor position
void homePosition(MotorConfig* motor) {

	setRPM(8, motorSetSteps); // (RPM, Driver Steps)
	setAcceleration(5.0f); // Set acceleration in steps per second^2

    // Move motor backward until limit switch is triggered
    while (HAL_GPIO_ReadPin(LIMIT_SW_GPIO_Port, LIMIT_SW_Pin) == GPIO_PIN_SET) {
    	// Calculate the new speed and step interval
		unsigned long stepInterval = computeNewSpeed();

		// Generate a step pulse with the calculated interval
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
		microDelay(stepInterval / 2);  // Half of the interval for the high pulse
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
		microDelay(stepInterval / 2);  // Half of the interval for the low pulse
    }
    HAL_Delay(300);
    // move motor forward 3mm or 480 steps

    HAL_GPIO_WritePin(GPIOA, DRIVE_ENB_Pin, GPIO_PIN_RESET);
    HomeMotorMove(motor, 160*10); // motor will move 3mm or 480 steps after hitting the limit switch, 1mm = 160 steps
    HAL_GPIO_WritePin(GPIOA, DRIVE_ENB_Pin, GPIO_PIN_SET);

    // Set encoder value to zero
    HAL_Delay(10);
    __disable_irq();

    encoderValue = 0;
    __enable_irq();
}

/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */

  // Define motor configurations
  MotorConfig motor1 = {GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_0};

  // Initialize motor position
   homePosition(&motor1);

  /* Infinite loop */
  for(;;)
  {
	  //Reading RMP value  from RPM holding register
	  rpm = Holding_Registers_Database[0];
	  if(rpm != prev_rpm)
	  {
	  setRPM(rpm, motorSetSteps); // (RPM, Steps)
	  prev_rpm = rpm;
	  }

	  //Reading acceleration value from acceleration holding register
	  acceleration = Holding_Registers_Database[1];
	  if(acceleration != prev_acceleration)
	  {
	   setAcceleration(acceleration);
	   prev_acceleration = acceleration;
	  }

    // Disable interrupts and read the encoder value
    //__disable_irq();
    currentPosition = encoderValue; // Read the latest encoder value
    //__enable_irq();

    input_distance = Holding_Registers_Database[4];


	// Calculate target position with floating-point division
	targetPosition = input_distance * (enc_val_in_1rev/mm_in_1rev); // in encoder value

	// Calculate position to move
	positionToMove = (long int)targetPosition - currentPosition; // in encoder value

	// Calculate steps to move
	stepsToMove = positionToMove * (motorSetSteps / (float)enc_val_in_1rev);

	if ((prev_input_distance != input_distance) || (prevstepsToMove != stepsToMove))
	{
	//Enable Drive
	HAL_GPIO_WritePin(GPIOA, DRIVE_ENB_Pin, GPIO_PIN_RESET);
	// Move motor
		motorMove(&motor1, stepsToMove);

	}



	if ((prev_input_distance == input_distance) || (prevstepsToMove == stepsToMove))
	{
		//Disable Drive
		HAL_GPIO_WritePin(GPIOA, DRIVE_ENB_Pin, GPIO_PIN_SET);
	}


	prev_input_distance = input_distance;
	prevstepsToMove = stepsToMove;

	osDelay(20);
  }
  /* USER CODE END StartMotorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
