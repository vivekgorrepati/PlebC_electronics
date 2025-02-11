/*
 * bno055.c
 *
 *  Created on: Aug 22, 2024
 *      Author: AKASH BEHRA
 */

#include "main.h"
#include "bno055.h"
char buffer[50];

extern I2C_HandleTypeDef hi2c1;


#define BNO055_I2C_ADDR          (0x28 << 1) // I2C address of BNO055 (adjust as needed)
#define BNO055_SYS_TRIGGER_REG   (0x3F)
#define BNO055_PWR_MODE_REG      (0x3E)
#define BNO055_OPR_MODE_REG      (0x3D)
#define BNO055_QUATERNION_DATA_W (0x20)
#define BNO055_CALIB_STAT_REG    (0x35)
#define BNO055_ACC_DATA_X_LSB    (0x08)
#define BNO055_ACC_DATA_X_MSB    (0x09)
#define BNO055_OPR_MODE_NDOF     (0x0C) // No Drift Orientation Fusion mode

float imuValues[4] = {0.0000, 0.0000, 0.0000, 0.0000};

HAL_StatusTypeDef BNO055_WriteReg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

HAL_StatusTypeDef BNO055_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t size) {
    return HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, size, 100);
}

HAL_StatusTypeDef BNO055_Init(void) {
    HAL_StatusTypeDef ret;

    // Reset the BNO055
    ret = BNO055_WriteReg(BNO055_SYS_TRIGGER_REG, 0x20);
    if (ret != HAL_OK) return ret;
    HAL_Delay(650); // Wait for the reset to complete

    // Set power mode to normal
    ret = BNO055_WriteReg(BNO055_PWR_MODE_REG, 0x00);
    if (ret != HAL_OK) return ret;
    HAL_Delay(10);

    // Set operation mode to NDOF
    ret = BNO055_WriteReg(BNO055_OPR_MODE_REG, BNO055_OPR_MODE_NDOF);
    if (ret != HAL_OK) return ret;
    HAL_Delay(20);

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadQuaternion(Quaternion *quat) {
    uint8_t buffer[8];
    HAL_StatusTypeDef ret;

    ret = BNO055_ReadRegs(BNO055_QUATERNION_DATA_W, buffer, 8);
    if (ret != HAL_OK) return ret;

    int16_t w_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t x_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t y_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t z_raw = (int16_t)((buffer[7] << 8) | buffer[6]);

    quat->w = w_raw / 16384.0f;  // 2^14 = 16384
    quat->x = x_raw / 16384.0f;
    quat->y = y_raw / 16384.0f;
    quat->z = z_raw / 16384.0f;

//    imuValues[0] = w_raw / 16384.0f;  // 2^14 = 16384
//    imuValues[1] = x_raw / 16384.0f;
//    imuValues[2] = y_raw / 16384.0f;
//    imuValues[3] = z_raw / 16384.0f;


    return HAL_OK;
}
