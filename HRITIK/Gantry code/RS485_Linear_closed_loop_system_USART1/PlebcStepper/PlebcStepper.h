/*
 * plebcStepper.h
 *
 *  Created on: 22-May-2024
 *      Author: Hritik Kumar
 */

#ifndef PLEBCSTEPPER_H_
#define PLEBCSTEPPER_H_

#include "stm32f4xx_hal.h"

// Structure to hold motor configuration
typedef struct {
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;
} MotorConfig;

// Function prototypes
void microDelay(uint16_t delay);
void setMaxSpeed(float speed);
void setAcceleration(float acceleration);
void setRPM(volatile float rpm, int steps);
unsigned long computeNewSpeed(void);
void motorRunForward(MotorConfig* motor);
void motorRunBackward(MotorConfig* motor);
void motorRunContinuous(MotorConfig* motor);
void HomeMotorMove(MotorConfig* motor, int steps_to_move);
void motorMove(MotorConfig* motor, int steps_to_move);
void motorStop(MotorConfig* motor);
void emergencyMotorStop(MotorConfig* motor);

#endif /* PLEBCSTEPPER_H_ */
