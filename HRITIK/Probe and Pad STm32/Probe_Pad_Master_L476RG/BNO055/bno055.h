/*
 * bno055.h
 *
 *  Created on: Aug 22, 2024
 *      Author: AKASH BEHRA
 */

#ifndef BNO055_H_
#define BNO055_H_

extern float imuValues[4];
// Define a structure for the floating-point quaternion
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/* BNO055 */
HAL_StatusTypeDef BNO055_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BNO055_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef BNO055_Init(void);
HAL_StatusTypeDef BNO055_ReadCalibStatus(uint8_t *calib_status);
HAL_StatusTypeDef BNO055_ReadAccel(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z);
HAL_StatusTypeDef BNO055_ReadQuaternion(Quaternion *quat);

#endif /* BNO055_H_ */
