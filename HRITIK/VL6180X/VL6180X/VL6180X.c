#include <VL6180X.h>

//float _gain = 1.0f;
//uint8_t _atime =100;

bool begin(VL6180X_t *sensor)
{
  if((getDeviceID(sensor)!=VL6180X_ID)){
    return false;
  }
  init(sensor);
  return true;
}

uint8_t getDeviceID(VL6180X_t *sensor)
{
    uint8_t deviceID = 0;
    HAL_StatusTypeDef status;

    // Read 1 byte from the VL6180X IDENTIFICATION_MODEL_ID register
    status = HAL_I2C_Mem_Read(sensor -> hi2c, VL6180X_ADDR, VL6180X_IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_16BIT, &deviceID, 1, HAL_MAX_DELAY);

    // Check if the read operation was successful
    if (status == HAL_OK)
    {
        return deviceID;  // Return the read device ID
    }
    else
    {
        // Handle the error (you can return 0 or some error code if necessary)
        return 0;
    }
}

void init(VL6180X_t *sensor)
{
  if(read(sensor,VL6180X_SYSTEM_FRESH_OUT_OF_RESET,1))
  {
    write8bit(sensor, 0x0207, 0x01);
    write8bit(sensor, 0x0208, 0x01);
    write8bit(sensor, 0x0096, 0x00);
    write8bit(sensor, 0x0097, 0xfd);
    write8bit(sensor, 0x00e3, 0x00);
    write8bit(sensor, 0x00e4, 0x04);
    write8bit(sensor, 0x00e5, 0x02);
    write8bit(sensor, 0x00e6, 0x01);
    write8bit(sensor, 0x00e7, 0x03);
    write8bit(sensor, 0x00f5, 0x02);
    write8bit(sensor, 0x00d9, 0x05);
    write8bit(sensor, 0x00db, 0xce);
    write8bit(sensor, 0x00dc, 0x03);
    write8bit(sensor, 0x00dd, 0xf8);
    write8bit(sensor, 0x009f, 0x00);
    write8bit(sensor, 0x00a3, 0x3c);
    write8bit(sensor, 0x00b7, 0x00);
    write8bit(sensor, 0x00bb, 0x3c);
    write8bit(sensor, 0x00b2, 0x09);
    write8bit(sensor, 0x00ca, 0x09);
    write8bit(sensor, 0x0198, 0x01);
    write8bit(sensor, 0x01b0, 0x17);
    write8bit(sensor, 0x01ad, 0x00);
    write8bit(sensor, 0x00ff, 0x05);
    write8bit(sensor, 0x0100, 0x05);
    write8bit(sensor, 0x0199, 0x05);
    write8bit(sensor, 0x01a6, 0x1b);
    write8bit(sensor, 0x01ac, 0x3e);
    write8bit(sensor, 0x01a7, 0x1f);
    write8bit(sensor, 0x0030, 0x00);
  }
  write8bit(sensor, VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);
  write8bit(sensor, VL6180X_SYSALS_ANALOGUE_GAIN, 0x46);
  write8bit(sensor, VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF);
  write8bit(sensor, VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63);
  write8bit(sensor, VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);
  write8bit(sensor, VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09);
  write8bit(sensor, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x31);
  write8bit(sensor, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x00);
  write8bit(sensor, VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x31);
  write8bit(sensor, VL6180X_INTERLEAVED_MODE_ENABLE, 0);
  write8bit(sensor, VL6180X_SYSTEM_MODE_GPIO1,0x20);
  write8bit(sensor, VL6180X_SYSTEM_FRESH_OUT_OF_RESET,0);
}

uint16_t read(VL6180X_t *sensor, uint16_t regAddr, uint8_t readNum)
{
    uint16_t value = 0;
    uint8_t data[2];  // Buffer for received data
    HAL_StatusTypeDef status;

    // Send the register address (high byte + low byte)
    uint8_t regAddrData[2] = {
        (uint8_t)(regAddr >> 8),   // High byte of register address
        (uint8_t)(regAddr & 0xFF)  // Low byte of register address
    };

    // Start transmission to send register address
    status = HAL_I2C_Master_Transmit(sensor -> hi2c, VL6180X_ADDR, regAddrData, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        // Handle the error if transmission fails
        return 0;
    }

    // Read data from the register
    status = HAL_I2C_Master_Receive(sensor -> hi2c, VL6180X_ADDR, data, readNum, 100);
    if (status != HAL_OK)
    {
        // Handle the error if reception fails
        return 0;
    }

    // Process the received data based on the number of bytes to read
    if (readNum == 1)
    {
        value = data[0];
    }
    else if (readNum == 2)
    {
        value = (data[1] << 8) | data[0];  // Combine two bytes into a 16-bit value
    }

    return value;
}

void write8bit(VL6180X_t *sensor, uint16_t regAddr, uint8_t value)
{
    uint8_t data[3];
    HAL_StatusTypeDef status;

    // First byte: high byte of the register address
    data[0] = (uint8_t)(regAddr >> 8);
    // Second byte: low byte of the register address
    data[1] = (uint8_t)(regAddr & 0xFF);
    // Third byte: value to write
    data[2] = value;

    // Write 3 bytes: register address (2 bytes) + value (1 byte)
    status = HAL_I2C_Master_Transmit(sensor -> hi2c, VL6180X_ADDR, data, 3, HAL_MAX_DELAY);

    // Check if the write operation was successful
    if (status != HAL_OK)
    {
        // Handle the error (for example, you can return or set a flag)
        // Here you can use HAL_Error_Handler() or any custom error handling
    }
}

float alsPoLLMeasurement(VL6180X_t *sensor) {
    ALSStartReg_t ALSStartReg;

    // Set the ALS start register configuration
    ALSStartReg.startstop = 1;  // Set startstop to 1 (start ALS measurement)
    ALSStartReg.select = 0;     // Select a mode (e.g., default mode)

    // Write to the sensor to start the ALS measurement
    write8bit(sensor, VL6180X_SYSALS_START, *((uint8_t*)(&ALSStartReg)));

    // Return the ALS measurement result
    return alsGetMeasurement(sensor);
}

float alsGetMeasurement(VL6180X_t *sensor)
{
  float value = read(sensor, VL6180X_RESULT_ALS_VAL,2);
  value  = (0.32*100*value)/((sensor -> _gain) * (sensor ->_atime));
  return value;
}

// Function to set ALS Gain
//uint8_t setALSGain(uint8_t gain) {
//    if (gain > 7) {
//        return 0;  // Return false
//    }
//
//    // Set the _gain variable based on gain selection
//    switch (gain) {
//        case VL6180X_ALS_GAIN_20:   _gain = 20.0f;  break;
//        case VL6180X_ALS_GAIN_10:   _gain = 10.0f;  break;
//        case VL6180X_ALS_GAIN_5:    _gain = 5.0f;   break;
//        case VL6180X_ALS_GAIN_2_5:  _gain = 2.5f;   break;
//        case VL6180X_ALS_GAIN_1_67: _gain = 1.67f;  break;
//        case VL6180X_ALS_GAIN_1_25: _gain = 1.25f;  break;
//        case VL6180X_ALS_GAIN_1:    _gain = 1.0f;   break;
//        case VL6180X_ALS_GAIN_40:   _gain = 40.0f;  break;
//    }
//
//    // Write the gain value to the sensor
//    write8bit(VL6180X_SYSALS_ANALOGUE_GAIN, gain);
//
//    return 1;  // Return true
//}

uint8_t rangePollMeasurement(VL6180X_t *sensor)
{
    uint8_t rangeStartReg = 0x01; // Equivalent to setting startstop = 1 and select = 0
    write8bit(sensor, VL6180X_SYSRANGE_START, rangeStartReg); // Start range measurement
    return rangeGetMeasurement(sensor); // Get measurement result
}
//uint8_t rangePollMeasurement()
//{
//    uint8_t rangeStartReg = 0x01;
//    write8bit(VL6180X_SYSRANGE_START, rangeStartReg); // Start measurement
//
//    // Wait for measurement to be ready
//    uint8_t status;
//    do {
//        status = read(VL6180X_RESULT_INTERRUPT_STATUS_GPIO, 1);
//    } while ((status & 0x04) == 0); // Wait until range measurement is complete
//
//    uint8_t range = rangeGetMeasurement();
//
//    // Clear interrupt flag to allow new measurement
//    write8bit(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
//
//    return range;
//}


uint8_t rangeGetMeasurement(VL6180X_t *sensor)
{
  uint8_t value = read(sensor, VL6180X_RESULT_RANGE_VAL,1);
  return value;
}

uint8_t getRangeResult(VL6180X_t *sensor)
{
  return read(sensor, VL6180X_RESULT_RANGE_STATUS,1)>>4;
}
