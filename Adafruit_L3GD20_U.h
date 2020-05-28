/*!
 * @file Adafruit_L3GD20_U.h
 */

#ifndef __L3GD20_H__
#define __L3GD20_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
#define L3GD20_ADDRESS (0x6B)     //!< L3gD20 I2C address; 1101011 in binary
#define L3GD20_POLL_TIMEOUT (100) //!< Maximum number of read attempts
#define L3GD20_ID (0xD4)          //!< L3GD20 ID
#define L3GD20H_ID (0xD7)         //!< L3GD20H ID
// Sesitivity values from the mechanical characteristics in the datasheet.
#define GYRO_SENSITIVITY_250DPS (0.00875F) //!< Sensitivity at 250 dps
#define GYRO_SENSITIVITY_500DPS (0.0175F)  //!< Sensitivity at 500 dps
#define GYRO_SENSITIVITY_2000DPS (0.070F)  //!< Sensitivity at 2000 dps
/*=========================================================================*/

/*!
 * @brief Registers
 */
typedef enum {                        // DEFAULT    TYPE
  GYRO_REGISTER_WHO_AM_I = 0x0F,      // 11010100   r
  GYRO_REGISTER_CTRL_REG1 = 0x20,     // 00000111   rw
  GYRO_REGISTER_CTRL_REG2 = 0x21,     // 00000000   rw
  GYRO_REGISTER_CTRL_REG3 = 0x22,     // 00000000   rw
  GYRO_REGISTER_CTRL_REG4 = 0x23,     // 00000000   rw
  GYRO_REGISTER_CTRL_REG5 = 0x24,     // 00000000   rw
  GYRO_REGISTER_REFERENCE = 0x25,     // 00000000   rw
  GYRO_REGISTER_OUT_TEMP = 0x26,      //            r
  GYRO_REGISTER_STATUS_REG = 0x27,    //            r
  GYRO_REGISTER_OUT_X_L = 0x28,       //            r
  GYRO_REGISTER_OUT_X_H = 0x29,       //            r
  GYRO_REGISTER_OUT_Y_L = 0x2A,       //            r
  GYRO_REGISTER_OUT_Y_H = 0x2B,       //            r
  GYRO_REGISTER_OUT_Z_L = 0x2C,       //            r
  GYRO_REGISTER_OUT_Z_H = 0x2D,       //            r
  GYRO_REGISTER_FIFO_CTRL_REG = 0x2E, // 00000000   rw
  GYRO_REGISTER_FIFO_SRC_REG = 0x2F,  //            r
  GYRO_REGISTER_INT1_CFG = 0x30,      // 00000000   rw
  GYRO_REGISTER_INT1_SRC = 0x31,      //            r
  GYRO_REGISTER_TSH_XH = 0x32,        // 00000000   rw
  GYRO_REGISTER_TSH_XL = 0x33,        // 00000000   rw
  GYRO_REGISTER_TSH_YH = 0x34,        // 00000000   rw
  GYRO_REGISTER_TSH_YL = 0x35,        // 00000000   rw
  GYRO_REGISTER_TSH_ZH = 0x36,        // 00000000   rw
  GYRO_REGISTER_TSH_ZL = 0x37,        // 00000000   rw
  GYRO_REGISTER_INT1_DURATION = 0x38  // 00000000   rw
} gyroRegisters_t;

/*!
 * @brief Optional speed settings
 */
typedef enum {
  GYRO_RANGE_250DPS = 250,
  GYRO_RANGE_500DPS = 500,
  GYRO_RANGE_2000DPS = 2000
} gyroRange_t;

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
/** Encapsulates a single raw data sample from the sensor. */
typedef struct gyroRawData_s {
  /** The X axis data. */
  int16_t x;
  /** The Y axis data. */
  int16_t y;
  /** The Z axis data. */
  int16_t z;
} gyroRawData_t;
/*=========================================================================*/

/**
 * Driver for the Adafruit L3GD20 3-Axis gyroscope.
 */
class Adafruit_L3GD20_Unified : public Adafruit_Sensor {
public:
  Adafruit_L3GD20_Unified(int32_t sensorID = -1);

  bool begin(gyroRange_t rng = GYRO_RANGE_250DPS, TwoWire *theWire = &Wire);
  void enableAutoRange(bool enabled);
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

  /** Raw sensor data from the last successful read event. */
  gyroRawData_t raw;

private:
  void write8(byte reg, byte value);
  byte read8(byte reg);
  gyroRange_t _range;
  int32_t _sensorID;
  bool _autoRangeEnabled;
};

/* Non Unified (old) driver for compatibility reasons */
typedef gyroRange_t l3gd20Range_t;         //!< Gyroscope range
typedef gyroRegisters_t l3gd20Registers_t; //!< Gyroscope registers

/**
 * Encapsulates a single XYZ data sample from the sensor.
 */
typedef struct l3gd20Data_s {
  /** Data from the X axis. */
  float x;
  /** Data from the Y axis. */
  float y;
  /** Data from the Z axis. */
  float z;
} l3gd20Data;

/// @private
class Adafruit_L3GD20 {
public:
  Adafruit_L3GD20(int8_t cs, int8_t mosi, int8_t miso, int8_t clk);
  Adafruit_L3GD20(void);

  bool begin(l3gd20Range_t rng = GYRO_RANGE_250DPS, byte addr = L3GD20_ADDRESS);
  void read(void);

  l3gd20Data data; // Last read will be available here

private:
  void write8(l3gd20Registers_t reg, byte value);
  byte read8(l3gd20Registers_t reg);
  uint8_t SPIxfer(uint8_t x);

  byte address;
  l3gd20Range_t range;
  int8_t _miso, _mosi, _clk, _cs;
};
#endif
