/*
 * loadcell.c
 *
 *  Created on: Aug 22, 2024
 *      Author: AKASH BEHRA
 */

#include "main.h"
#include "loadcell.h"

#define DT_PIN 		GPIO_PIN_10
#define DT_PORT 	GPIOA
#define SCK_PIN 	GPIO_PIN_3
#define SCK_PORT 	GPIOB

extern TIM_HandleTypeDef htim1;

uint32_t tare;
float knownOriginal = 55.0;  // in milli gram
float knownHX711 = -2757821.0;

/*--------------- Load Cell -----------------------------*/
void microDelay(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/* Function to read data from HX711 -----------------------------------------*/

int32_t getHX711(void)
{
  uint32_t data = 0;
  uint32_t startTime = HAL_GetTick();

  /* Wait for DT pin to go low */
  while (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
	if (HAL_GetTick() - startTime > 200) // Timeout if data line is not ready
	  return 0;
  }

  /* Read 24 bits of data from HX711 */
  for (int8_t i = 0; i < 24; i++)
  {
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
	microDelay(1); // Ensure enough time for data to settle
	data = data << 1;
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
	microDelay(1); // Ensure enough time between toggles
	if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
	  data++;
  }

  /* Convert to signed 24-bit integer */
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  microDelay(1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  microDelay(1);

  return data;
}

int weigh()
{
  int32_t  total = 0;
  int32_t  samples = 1;
  int milligram;
  float coefficient;

  for(uint16_t i=0 ; i<samples ; i++)
  {
	  total += getHX711();
  }
  int32_t average = (int32_t)(total / samples);
  coefficient = knownOriginal / knownHX711;
  milligram = (int)(average-tare)*coefficient;
  return milligram;
}

void setTare(void)
{
	int32_t total = 0;
	int32_t samples = 50; // Take multiple samples for averaging

	for (uint16_t i = 0; i < samples; i++)
	{
		total += getHX711();
		HAL_Delay(100); // Delay between samples
	}

	tare = total / samples;
}
