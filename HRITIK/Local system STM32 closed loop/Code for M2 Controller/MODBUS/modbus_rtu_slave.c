/*
 * modbus_rtu_slave.c
 *
 *  Created on: Jun 10, 2024
 *      Author: HRITIK KUMAR
 */

#include "main.h"

#include "modbus_rtu_slave.h"
#include "string.h"

// Data buffers for UART communication
uint8_t RxData[256];
uint8_t TxData[256];


// Databases for holding registers and coils
// Holding Registers (40001 - 49999): These are registers used for read and write operations.
/*
{
    RPM[0], acceleration[1],  Reserved(2),  Motor Steps(3),  Input angle(4),
    motor stop(5),  emergency stop(6),  none,  none,  none,
    none, none, none, none, none, none, none, none, none, none,
    none, none, none, none, none, none, none, none, none, none,
    none, none, none, none, none, none, none, none, none, none,
    none, none, none, none, none, none, none, none, none, none
};
*/
uint16_t Holding_Registers_Database[MAX_HOLDING_REGISTERS] = {
    150,    2,	  0,  6400,  0,     0,     0,     7777,  8888,  9999,
    12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,
    21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,
    31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,
    45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029
};

// Input Registers (30001 - 39999): These are read-only registers.
/*
 * {
		Current Encoder value,  Current angle,  Current rpm,  Current acceleration,  none,  none,  none,  none,  none,  none,
		none, none, none, none, none, none, none, none, none, none,
		none, none, none, none, none, none, none, none, none, none,
		none, none, none, none, none, none, none, none, none, none,
		none, none, none, none, none, none, none, none, none, none
	};
*/
uint16_t Input_Registers_Database[MAX_INPUT_REGISTERS] = {
    00000, 00000, 00000, 00000, 00000, 55555,  6666,  7777,  8888,  9999,
    12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,
    21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,
    31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,
    45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029
};
// Coils (00001 - 09999): These are single-bit registers used for read and write operations.
uint8_t Coils_Database[MAX_COILS / 8] = {
    0b01001001, 0b10011100, 0b10101010, 0b01010101, 0b11001100,
    0b10100011, 0b01100110, 0b10101111, 0b01100000, 0b10111100,
    0b11001100, 0b01101100, 0b01010011, 0b11111111, 0b00000000,
    0b01010101, 0b00111100, 0b00001111, 0b11110000, 0b10001111,
    0b01010100, 0b10011001, 0b11111000, 0b00001101, 0b00101010
};

uint8_t Discrete_Inputs[MAX_DISCRETE_INPUTS] = {
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0
};

void sendData(uint8_t *data, int size) {
    // Enable transmission by setting DE and RE pins high
    HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_SET);

    // Transmit data
    uint16_t crc = crc16(data, size);
    data[size] = crc & 0xFF;
    data[size + 1] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, data, size + 2, 1000);

    // Disable transmission by setting DE and RE pins low
    HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_RESET);

}


uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbusException(uint8_t exceptionCode) {
    TxData[0] = RxData[0];
    TxData[1] = RxData[1] | 0x80;
    TxData[2] = exceptionCode;
    sendData(TxData, 3);
}

uint8_t readHoldingRegs(uint16_t startAddr, uint16_t numRegs) {
    if ((numRegs < 1) || (numRegs > MAX_HOLDING_REGISTERS)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numRegs - 1;
    if (endAddr > MAX_HOLDING_REGISTERS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    memset(TxData, '\0', 256);
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = numRegs * 2;

    int indx = 3;

    for (int i = 0; i < numRegs; i++) {
        TxData[indx++] = (Holding_Registers_Database[startAddr] >> 8) & 0xFF; // High byte
        TxData[indx++] = Holding_Registers_Database[startAddr] & 0xFF;        // Low byte
        startAddr++;
    }

    sendData(TxData, indx);
    return 1;
}

uint8_t readInputRegs(uint16_t startAddr, uint16_t numRegs) {
    if ((numRegs < 1) || (numRegs > MAX_INPUT_REGISTERS)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numRegs - 1;
    if (endAddr > MAX_INPUT_REGISTERS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    memset(TxData, '\0', 256);
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = numRegs * 2;

    int indx = 3;

    for (int i = 0; i < numRegs; i++) {
        TxData[indx++] = (Input_Registers_Database[startAddr] >> 8) & 0xFF; // High byte
        TxData[indx++] = Input_Registers_Database[startAddr] & 0xFF;        // Low byte
        startAddr++;
    }

    sendData(TxData, indx);
    return 1;
}


uint8_t readCoils(uint16_t startAddr, uint16_t numCoils) {
    if ((numCoils < 1) || (numCoils > 2000)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numCoils - 1;
    if (endAddr > 199) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    memset(TxData, '\0', 256);
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);

    int indx = 3;
    int startByte = startAddr / 8;
    uint16_t bitPosition = startAddr % 8;
    int indxPosition = 0;

    for (int i = 0; i < numCoils; i++) {
        TxData[indx] |= ((Coils_Database[startByte] >> bitPosition) & 0x01) << indxPosition;
        indxPosition++;
        bitPosition++;
        if (indxPosition > 7) {
            indxPosition = 0;
            indx++;
        }
        if (bitPosition > 7) {
            bitPosition = 0;
            startByte++;
        }
    }

    if (numCoils % 8 != 0) indx++;
    sendData(TxData, indx);
    return 1;
}

uint8_t writeSingleHoldingReg(uint16_t writeAddr, uint16_t regValue) {
    if (writeAddr >= MAX_HOLDING_REGISTERS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    Holding_Registers_Database[writeAddr] = regValue;

    // Construct the response message
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = RxData[2];
    TxData[3] = RxData[3];
    TxData[4] = RxData[4];
    TxData[5] = RxData[5];

    // Send the response message
    sendData(TxData, 6);
    return 1;
}


uint8_t writeHoldingRegs(uint16_t startAddr, uint16_t numRegs, uint16_t *data) {
    if ((numRegs < 1) || (numRegs > 123)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numRegs - 1;
    if (endAddr > 49) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    for (int i = 0; i < numRegs; i++) {
        Holding_Registers_Database[startAddr++] = data[i];
    }

    // Construct the response message
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = RxData[2];
    TxData[3] = RxData[3];
    TxData[4] = RxData[4];
    TxData[5] = RxData[5];

    // Send the response message
    sendData(TxData, 6);
    return 1;
}




uint8_t writeSingleCoil(uint16_t writeAddr, uint8_t coilValue) {
    if (writeAddr > 199) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    int bytePos = writeAddr / 8;
    int bitPos = writeAddr % 8;

    if ((coilValue == 0xFF) && (RxData[5] == 0x00)) {
        Coils_Database[bytePos] |= (1 << bitPos);
    } else if ((coilValue == 0x00) && (RxData[5] == 0x00)) {
        Coils_Database[bytePos] &= ~(1 << bitPos);
    } else {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = RxData[2];
    TxData[3] = RxData[3];
    TxData[4] = coilValue;  // Changed to coilValue
    TxData[5] = 0x00;        // Assuming the sixth byte is always 0x00 in this function

    sendData(TxData, 6);
    return 1;
}


uint8_t writeMultiCoils(uint16_t startAddr, uint16_t numCoils, uint8_t *coilData) {
    if ((numCoils < 1) || (numCoils > 1968)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numCoils - 1;
    if (endAddr > 199) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    int startByte = startAddr / 8;
    uint16_t bitPosition = startAddr % 8;
    int byteCount = numCoils / 8 + ((numCoils % 8) > 0 ? 1 : 0);
    int indx = 0;
    for (int i = 0; i < byteCount; i++) {
        for (int j = 0; j < 8; j++) {
            if (indx >= numCoils) break;
            if ((coilData[i] >> j) & 1) {
                Coils_Database[startByte] |= (1 << bitPosition);
            } else {
                Coils_Database[startByte] &= ~(1 << bitPosition);
            }
            bitPosition++;
            if (bitPosition > 7) {
                bitPosition = 0;
                startByte++;
            }
            indx++;
        }
    }

    // Construct the response message
    TxData[0] = Slave_ID;
    TxData[1] = RxData[1];
    TxData[2] = RxData[2];
    TxData[3] = RxData[3];
    TxData[4] = RxData[4];
    TxData[5] = RxData[5];

    // Send the response message
    sendData(TxData, 6);
    return 1;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {

        if (Size < 1 || Size > RX_BUFFER_SIZE) {
            // Restart UART reception for next packet
            HAL_UARTEx_ReceiveToIdle_IT(huart, RxData, RX_BUFFER_SIZE);
            return;
        }

        // Disable DE and RE pins for reception
        HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_RESET);

        // Check Slave ID
        if (RxData[0] != Slave_ID) {
            // Restart UART reception for next packet
            HAL_UARTEx_ReceiveToIdle_IT(huart, RxData, RX_BUFFER_SIZE);
            return;
        }

        // Calculate and Check CRC
        uint16_t crc = ((RxData[Size - 1] << 8) | RxData[Size - 2]);
        if (crc != crc16(RxData, Size - 2)) {
            // Restart UART reception for next packet
            HAL_UARTEx_ReceiveToIdle_IT(huart, RxData, RX_BUFFER_SIZE);
            return;
        }

        // Extract start address and number of registers/coils
        uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
        uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);

        // Enable DE and RE pins before transmitting response
        HAL_GPIO_WritePin(DE_RE_ENB_GPIO_Port, DE_RE_ENB_Pin, GPIO_PIN_SET);

        // Handle the Modbus function codes
        switch (RxData[1]) {
            case 1:  // Function code for reading coils
                readCoils(startAddr, numRegs);
                break;
            case 3:  // Function code for reading holding registers
                readHoldingRegs(startAddr, numRegs);
                break;
            case 4:  // Function code for reading input registers
                readInputRegs(startAddr, numRegs);
                break;
            case 5:  // Function code for writing a single coil
                writeSingleCoil(startAddr, RxData[4]);
                break;
            case 6:  // Function code for writing a single holding register
                writeSingleHoldingReg(startAddr, (RxData[4] << 8) | RxData[5]);
                break;
            case 15: // Function code for writing multiple coils
                writeMultiCoils(startAddr, numRegs, &RxData[7]);
                break;
            case 16: // Function code for writing holding registers
                writeHoldingRegs(startAddr, numRegs, (uint16_t *)&RxData[6]);
                break;
            default:
                modbusException(ILLEGAL_FUNCTION);
                break;
        }

        // Restart UART reception
        HAL_UARTEx_ReceiveToIdle_IT(huart, RxData, RX_BUFFER_SIZE);
    }
}
