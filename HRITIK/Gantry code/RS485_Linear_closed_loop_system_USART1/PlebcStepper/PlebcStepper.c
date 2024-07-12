#include <PlebcStepper.h>
#include <modbus_rtu_slave.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" // or your specific HAL header

// Global variables
volatile float _speed = 0.0;
volatile float _acceleration = 0.0;
volatile float _maxSpeed = 0.0;
volatile long _stepInterval = 0;

volatile int input = 0;
volatile int prev_input = 0;

bool motorStopReg = false;
bool emergencyMotorStopReg = false;

// External TIM handle declared in main.c
extern TIM_HandleTypeDef htim1;

void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}


void setMaxSpeed(float speed) {
    _maxSpeed = speed;
}

void setAcceleration(float acceleration) {
    _acceleration = acceleration;
}

void setRPM(volatile float rpm, int steps) {
    volatile float speed = (rpm * steps) / 60.0f;
    _maxSpeed = speed;

}

unsigned long computeNewSpeed() {
    if (_speed < _maxSpeed) {
        _speed += _acceleration;
        if (_speed > _maxSpeed) {
            _speed = _maxSpeed;
        }
    } else if (_speed > _maxSpeed) {
        _speed -= _acceleration;
        if (_speed < _maxSpeed) {
            _speed = _maxSpeed;
        }
    }

    // Prevent division by zero
    if (_speed <= 0) {
        _speed = 1; // Or any minimum non-zero value suitable for your application
    }

    // Calculate step interval in microseconds
    _stepInterval = (long)(1000000.0 / _speed);

    return _stepInterval;
}

void motorRunForward(MotorConfig* motor) {
// Set direction pin for forward movement
	HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_SET);

	while (1) {
		// Calculate the new speed and step interval
		unsigned long stepInterval = computeNewSpeed();

		// Generate a step pulse with the calculated interval
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
		microDelay(stepInterval / 2);  // Half of the interval for the high pulse
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
		microDelay(stepInterval / 2);  // Half of the interval for the low pulse

		// Optional: Implement a way to exit the loop if necessary
	}
}

void motorRunBackward(MotorConfig* motor) {
// Set direction pin for forward movement
	HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_RESET);

	while (1) {
		// Calculate the new speed and step interval
		unsigned long stepInterval = computeNewSpeed();

		// Generate a step pulse with the calculated interval
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
		microDelay(stepInterval / 2);  // Half of the interval for the high pulse
		HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
		microDelay(stepInterval / 2);  // Half of the interval for the low pulse

		// Optional: Implement a way to exit the loop if necessary
	}
}

void motorRunContinuous(MotorConfig* motor) {
    // Set direction pin for forward movement
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_SET);

    while (1) {
        // Calculate the new speed and step interval
        unsigned long stepInterval = computeNewSpeed();

        // Generate a step pulse with the calculated interval
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
        microDelay(stepInterval / 2);  // Half of the interval for the high pulse
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepInterval / 2);  // Half of the interval for the low pulse

        // Optional: Implement a way to exit the loop if necessary
    }
}


void motorMove(MotorConfig* motor, int steps_to_move) {

    // Set direction based on the steps_to_move value
    if (steps_to_move > 0) {
        HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_SET);  // Set direction for forward movement
    } else {
        HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_RESET);  // Set direction for backward movement
        steps_to_move = -steps_to_move; // Convert steps_to_move to positive value
    }

    for (int i = 0; i < steps_to_move; i++) {
        motorStopReg = (bool) Holding_Registers_Database[5];
        emergencyMotorStopReg = (bool) Holding_Registers_Database[6];

        if (motorStopReg) {
            //printf("Motor stopped by motorStopReg.\n");
            motorStop(motor);
            break;  // Exit the loop on motor stop condition
        }

        if (emergencyMotorStopReg) {
            //printf("Motor stopped by emergencyMotorStopReg.\n");
            emergencyMotorStop(motor);
            break;  // Exit the loop on emergency stop condition
        }

        // Calculate the new speed and step interval
        unsigned long stepInterval = computeNewSpeed();

        // Generate a step pulse with the calculated interval
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
        microDelay(stepInterval / 2);  // High pulse duration
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepInterval / 2);  // Low pulse duration
    }

    // Resetting stop flags after movement
    Holding_Registers_Database[5] = 0;
    Holding_Registers_Database[6] = 0;

}


void motorStop(MotorConfig* motor) {
    while (_speed > 0) {
        _speed -= _acceleration;
        if (_speed < 0) {
            _speed = 0;
        }

        // Calculate the new step interval for deceleration
        unsigned long stepInterval = computeNewSpeed();

        // Generate a step pulse with the calculated interval
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_SET);
        microDelay(stepInterval / 2);  // Half of the interval for the high pulse
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepInterval / 2);  // Half of the interval for the low pulse
    }
    	// step pin is low after stopping
        HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);
        Holding_Registers_Database[4] = Input_Registers_Database[1];
}



void emergencyMotorStop(MotorConfig* motor) {
    // Ensure the step pin is low
    HAL_GPIO_WritePin(motor->STEP_PORT, motor->STEP_PIN, GPIO_PIN_RESET);

    // Optionally, you can set the direction pin to a known state (e.g., low)
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_RESET);


    Holding_Registers_Database[4] = Input_Registers_Database[1];

}
