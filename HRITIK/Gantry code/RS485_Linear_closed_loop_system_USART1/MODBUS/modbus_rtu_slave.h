#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H


#include <stdint.h>
#include "stm32f4xx_hal.h"

// Modbus RTU Slave ID
#define SLAVE_ID 1 		// Z-axis
//#define SLAVE_ID 2		// X-axis
//#define SLAVE_ID 3		// Y-axis

#define RX_BUFFER_SIZE 256


// Modbus exception codes
#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03

// Maximum number of holding registers and coils
#define MAX_HOLDING_REGISTERS 50
#define MAX_COILS 			  200
#define MAX_DISCRETE_INPUTS   100
#define MAX_INPUT_REGISTERS   50

// External UART handle (replace with your actual UART handle)
extern UART_HandleTypeDef huart1;

// Function declarations
void sendData(uint8_t *data, int size);
uint16_t crc16(const uint8_t *data, uint16_t len);
void modbusException(uint8_t exceptionCode);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

uint8_t readHoldingRegs(uint16_t startAddr, uint16_t numRegs);
uint8_t readInputRegs(uint16_t startAddr, uint16_t numRegs);
uint8_t readCoils(uint16_t startAddr, uint16_t numCoils);
uint8_t writeHoldingRegs(uint16_t startAddr, uint16_t numRegs, uint16_t *data);
uint8_t writeSingleCoil(uint16_t writeAddr, uint8_t coilValue);
uint8_t writeMultiCoils(uint16_t startAddr, uint16_t numCoils, uint8_t *coilValues);

// External data buffers (adjust as needed for your use case)
extern uint8_t RxData[256];
extern uint8_t TxData[256];
extern uint16_t Holding_Registers_Database[MAX_HOLDING_REGISTERS];
extern uint16_t Input_Registers_Database[MAX_INPUT_REGISTERS];
extern uint8_t Coils_Database[MAX_COILS / 8];

#endif // MODBUS_RTU_SLAVE_H



