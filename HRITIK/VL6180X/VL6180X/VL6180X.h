#ifndef VL6180X_H_
#define VL6180X_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;




typedef struct {
    I2C_HandleTypeDef *hi2c;  // Pointer to I2C instance
    float _gain;               // ALS gain
    uint8_t _atime;              // ALS integration time
} VL6180X_t;


#define VL6180X_ADDR 0x29 << 1   // VL6180X I2C address (shifted for HAL)

#define VL6180X_ID          0xB4

//extern float _gain;  // Declare as external
//extern uint8_t _atime; // Declare as external


// Structure to represent the ALS Start Register (you'll need to define it based on the datasheet)
typedef struct {
    uint8_t startstop : 1;
    uint8_t select : 1;
    uint8_t reserved : 6;
} ALSStartReg_t;

/*---------------------Register Address-------------------*/
//Device model identification number：0XB4
#define VL6180X_IDENTIFICATION_MODEL_ID               0x000

#define VL6180X_SYSTEM_MODE_GPIO0                     0X010
#define VL6180X_SYSTEM_MODE_GPIO1                     0X011
/*
   * SYSTEM__MODE_GPIO1
   * -------------------------------------------------------------------------------------------------
   * |    b7    |    b6    |       b5        |    b4    |    b3    |    b2    |    b1     |    b0    |
   * -------------------------------------------------------------------------------------------------
   * |      reversed       |     polarity    |                   select                   | reversed |
   * -------------------------------------------------------------------------------------------------
   *
   *
*/

#define VL6180X_SYSTEM_INTERRUPT_CLEAR                0x015
/*
   * SYSTEM__INTERRUPT_CLEAR
   * ---------------------------------------------------------------------------------------------
   * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * ---------------------------------------------------------------------------------------------
   * |                         reversed                        |          int_clear_sig          |
   * ---------------------------------------------------------------------------------------------
   *
   *
*/

#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET             0x016

#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD         0x017

#define VL6180X_SYSRANGE_START                        0x018

#define VL6180X_SYSALS_INTEGRATION_PERIOD             0x040

#define VL6180X_RESULT_RANGE_STATUS                   0x04D
#define VL6180X_RESULT_ALS_STATUS                     0x04E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO          0x04F
#define VL6180X_RESULT_ALS_VAL                        0x050
#define VL6180X_RESULT_RANGE_VAL                      0x062



#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD       0x10A


#define VL6180X_FIRMWARE_RESULT_SCALER                0x120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS              0x212
#define VL6180X_INTERLEAVED_MODE_ENABLE               0x2A3

#define VL6180X_ID                                    0xB4
#define VL6180X_ALS_GAIN_20                           0x00
#define VL6180X_ALS_GAIN_10                           0x01
#define VL6180X_ALS_GAIN_5                            0x02
#define VL6180X_ALS_GAIN_2_5                          0x03
#define VL6180X_ALS_GAIN_1_67                         0x04
#define VL6180X_ALS_GAIN_1_25                         0x05
#define VL6180X_ALS_GAIN_1                            0x06
#define VL6180X_ALS_GAIN_40                           0x07

#define VL6180X_NO_ERR                                0x00
#define VL6180X_EARLY_CONV_ERR                        0x06
#define VL6180X_MAX_CONV_ERR                          0x07
#define VL6180X_IGNORE_ERR                            0x08
#define VL6180X_MAX_S_N_ERR                           0x0B
#define VL6180X_RAW_Range_UNDERFLOW_ERR               0x0C
#define VL6180X_RAW_Range_OVERFLOW_ERR                0x0D
#define VL6180X_Range_UNDERFLOW_ERR                   0x0E
#define VL6180X_Range_OVERFLOW_ERR                    0x0F

#define VL6180X_DIS_INTERRUPT        0
#define VL6180X_HIGH_INTERRUPT       1
#define VL6180X_LOW_INTERRUPT        2

#define VL6180X_INT_DISABLE          0
#define VL6180X_LEVEL_LOW            1
#define VL6180X_LEVEL_HIGH           2
#define VL6180X_OUT_OF_WINDOW        3
#define VL6180X_NEW_SAMPLE_READY     4

//High Threshold value for ALS comparison. Range 0-65535 codes.
#define VL6180X_SYSALS_THRESH_HIGH                    0x03A
// Low Threshold value for ALS comparison. Range 0-65535 codes.
#define VL6180X_SYSALS_THRESH_LOW                     0x03C
//Time delay between measurements in ALS continuous mode. Range 0-254 (0 = 10ms). Step size = 10ms.
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD        0x03E

#define VL6180X_SYSALS_ANALOGUE_GAIN                  0x03F
/*
   * SYSALS_ANALOGUE_GAIN
   * ---------------------------------------------------------------------------------------------
   * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * ---------------------------------------------------------------------------------------------
   * |                         reversed                        |   sysals__analogue_gain_light   |
   * ---------------------------------------------------------------------------------------------
   *
   *
*/
// High Threshold value for ranging comparison. Range 0-255mm.
#define VL6180X_SYSRANGE_THRESH_HIGH                  0x019

//Low Threshold value for ranging comparison. Range 0-255mm.
#define VL6180X_SYSRANGE_THRESH_LOW                   0x01A

// Time delay between measurements in Ranging continuous mode. Range 0-254 (0 = 10ms). Step size = 10ms.
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD      0x01B

//Maximum time to run measurement in Ranging modes.Range 1 - 63 ms
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME         0x01C

#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE   0x022
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT       0x02C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES          0x02D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE              0x02E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE              0x031

#define VL6180X_SYSALS_START                          0x038

/*
   * SYSALS__START
   * -----------------------------------------------------------------------------------------------
   * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |     b0     |
   * -----------------------------------------------------------------------------------------------
   * |                             reversed                               |   select  | startstop  |
   * -----------------------------------------------------------------------------------------------
   *
   *
   *
*/

#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO          0x014
/*
   * SYSTEM__INTERRUPT_CONFIG_GPIO
   * ---------------------------------------------------------------------------------------------
   * |    b7    |    b6    |       b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * ---------------------------------------------------------------------------------------------
   * |      reversed       |             als_int_mode          |         range_int_mode          |
   * ---------------------------------------------------------------------------------------------
*/

// Integration period for ALS mode. 1 code = 1 ms (0 = 1 ms). Recommended setting is 100 ms (0x63).
#define VL6180X_SYSALS_INTEGRATION_PERIOD             0x040

#define VL6180X_RESULT_RANGE_STATUS                   0x04D
#define VL6180X_RESULT_ALS_STATUS                     0x04E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO          0x04F
#define VL6180X_RESULT_ALS_VAL                        0x050
#define VL6180X_RESULT_RANGE_VAL                      0x062



#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD       0x10A


#define VL6180X_FIRMWARE_RESULT_SCALER                0x120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS              0x212
#define VL6180X_INTERLEAVED_MODE_ENABLE               0x2A3

#define VL6180X_ID                                    0xB4
#define VL6180X_ALS_GAIN_20                           0x00
#define VL6180X_ALS_GAIN_10                           0x01
#define VL6180X_ALS_GAIN_5                            0x02
#define VL6180X_ALS_GAIN_2_5                          0x03
#define VL6180X_ALS_GAIN_1_67                         0x04
#define VL6180X_ALS_GAIN_1_25                         0x05
#define VL6180X_ALS_GAIN_1                            0x06
#define VL6180X_ALS_GAIN_40                           0x07

#define VL6180X_NO_ERR                                0x00
#define VL6180X_EARLY_CONV_ERR                        0x06
#define VL6180X_MAX_CONV_ERR                          0x07
#define VL6180X_IGNORE_ERR                            0x08
#define VL6180X_MAX_S_N_ERR                           0x0B
#define VL6180X_RAW_Range_UNDERFLOW_ERR               0x0C
#define VL6180X_RAW_Range_OVERFLOW_ERR                0x0D
#define VL6180X_Range_UNDERFLOW_ERR                   0x0E
#define VL6180X_Range_OVERFLOW_ERR                    0x0F

#define VL6180X_DIS_INTERRUPT        0
#define VL6180X_HIGH_INTERRUPT       1
#define VL6180X_LOW_INTERRUPT        2

#define VL6180X_INT_DISABLE          0
#define VL6180X_LEVEL_LOW            1
#define VL6180X_LEVEL_HIGH           2
#define VL6180X_OUT_OF_WINDOW        3
#define VL6180X_NEW_SAMPLE_READY     4

bool begin(VL6180X_t *sensor);
uint8_t getDeviceID(VL6180X_t *sensor);
void init(VL6180X_t *sensor);
uint16_t read(VL6180X_t *sensor, uint16_t regAddr, uint8_t readNum);
void write8bit(VL6180X_t *sensor, uint16_t regAddr, uint8_t value);
float alsPoLLMeasurement(VL6180X_t *sensor) ;
float alsGetMeasurement(VL6180X_t *sensor);

uint8_t rangePollMeasurement(VL6180X_t *sensor);
uint8_t rangeGetMeasurement(VL6180X_t *sensor);
uint8_t getRangeResult(VL6180X_t *sensor);

#endif
