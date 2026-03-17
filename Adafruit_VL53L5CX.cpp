/*!
 * @file Adafruit_VL53L5CX.cpp
 *
 * Arduino library for the ST VL53L5CX 8x8 Time-of-Flight sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_VL53L5CX.h"

/*!
 * @brief Constructor
 */
Adafruit_VL53L5CX::Adafruit_VL53L5CX() {}

/*!
 * @brief Destructor
 */
Adafruit_VL53L5CX::~Adafruit_VL53L5CX() {
  if (_i2c_dev) {
    delete _i2c_dev;
  }
}

/*!
 * @brief Initialize the sensor
 * @param address I2C address (default 0x29)
 * @param wire Pointer to Wire instance
 * @return true on success, false on failure
 */
bool Adafruit_VL53L5CX::begin(uint8_t address, TwoWire *wire) {
  if (_i2c_dev) {
    delete _i2c_dev;
  }

  _i2c_dev = new Adafruit_I2CDevice(address, wire);
  if (!_i2c_dev->begin()) {
    return false;
  }

  // Set up the platform struct for ST driver
  _config.platform.address = address;
  _config.platform.i2c_dev = _i2c_dev;

  // Check sensor is alive
  uint8_t isAlive = 0;
  uint8_t status = vl53l5cx_is_alive(&_config, &isAlive);
  if (status != VL53L5CX_STATUS_OK || !isAlive) {
    return false;
  }

  // Initialize the sensor (loads firmware, takes ~10 seconds!)
  status = vl53l5cx_init(&_config);
  if (status != VL53L5CX_STATUS_OK) {
    return false;
  }

  _initialized = true;
  return true;
}

/*!
 * @brief Start ranging
 * @return true on success
 */
bool Adafruit_VL53L5CX::startRanging(void) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_start_ranging(&_config) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Stop ranging
 * @return true on success
 */
bool Adafruit_VL53L5CX::stopRanging(void) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_stop_ranging(&_config) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Check if new data is ready
 * @return true if data is ready to read
 */
bool Adafruit_VL53L5CX::isDataReady(void) {
  if (!_initialized) {
    return false;
  }
  uint8_t ready = 0;
  if (vl53l5cx_check_data_ready(&_config, &ready) != VL53L5CX_STATUS_OK) {
    return false;
  }
  return (ready != 0);
}

/*!
 * @brief Get ranging data from the sensor
 * @param results Pointer to results structure to fill
 * @return true on success
 */
bool Adafruit_VL53L5CX::getRangingData(VL53L5CX_ResultsData *results) {
  if (!_initialized || !results) {
    return false;
  }
  return (vl53l5cx_get_ranging_data(&_config, results) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Set resolution (16 for 4x4 or 64 for 8x8)
 * @param resolution 16 or 64
 * @return true on success
 */
bool Adafruit_VL53L5CX::setResolution(uint8_t resolution) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_resolution(&_config, resolution) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current resolution
 * @return Resolution (16 or 64), or 0 on error
 */
uint8_t Adafruit_VL53L5CX::getResolution(void) {
  if (!_initialized) {
    return 0;
  }
  uint8_t resolution = 0;
  if (vl53l5cx_get_resolution(&_config, &resolution) != VL53L5CX_STATUS_OK) {
    return 0;
  }
  return resolution;
}

/*!
 * @brief Set ranging frequency in Hz
 * @param frequency_hz Frequency (1-60 Hz depending on resolution)
 * @return true on success
 */
bool Adafruit_VL53L5CX::setRangingFrequency(uint8_t frequency_hz) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_ranging_frequency_hz(&_config, frequency_hz) ==
          VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current ranging frequency
 * @return Frequency in Hz, or 0 on error
 */
uint8_t Adafruit_VL53L5CX::getRangingFrequency(void) {
  if (!_initialized) {
    return 0;
  }
  uint8_t freq = 0;
  if (vl53l5cx_get_ranging_frequency_hz(&_config, &freq) !=
      VL53L5CX_STATUS_OK) {
    return 0;
  }
  return freq;
}

/*!
 * @brief Set integration time in milliseconds
 * @param time_ms Integration time (2-1000ms)
 * @return true on success
 */
bool Adafruit_VL53L5CX::setIntegrationTime(uint32_t time_ms) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_integration_time_ms(&_config, time_ms) ==
          VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current integration time
 * @return Integration time in ms, or 0 on error
 */
uint32_t Adafruit_VL53L5CX::getIntegrationTime(void) {
  if (!_initialized) {
    return 0;
  }
  uint32_t time_ms = 0;
  if (vl53l5cx_get_integration_time_ms(&_config, &time_ms) !=
      VL53L5CX_STATUS_OK) {
    return 0;
  }
  return time_ms;
}

/*!
 * @brief Set sharpener percentage (0=off, 1-99=amount)
 * @param percent Sharpener percentage
 * @return true on success
 */
bool Adafruit_VL53L5CX::setSharpenerPercent(uint8_t percent) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_sharpener_percent(&_config, percent) ==
          VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current sharpener percentage
 * @return Sharpener percent, or 0xFF on error
 */
uint8_t Adafruit_VL53L5CX::getSharpenerPercent(void) {
  if (!_initialized) {
    return 0xFF;
  }
  uint8_t percent = 0;
  if (vl53l5cx_get_sharpener_percent(&_config, &percent) !=
      VL53L5CX_STATUS_OK) {
    return 0xFF;
  }
  return percent;
}

/*!
 * @brief Set target order (closest or strongest first)
 * @param order VL53L5CX_TARGET_ORDER_CLOSEST (1) or
 *              VL53L5CX_TARGET_ORDER_STRONGEST (2)
 * @return true on success
 */
bool Adafruit_VL53L5CX::setTargetOrder(uint8_t order) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_target_order(&_config, order) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current target order
 * @return VL53L5CX_TARGET_ORDER_CLOSEST (1) or
 *         VL53L5CX_TARGET_ORDER_STRONGEST (2), or 0 on error
 */
uint8_t Adafruit_VL53L5CX::getTargetOrder(void) {
  if (!_initialized) {
    return 0;
  }
  uint8_t order = 0;
  if (vl53l5cx_get_target_order(&_config, &order) != VL53L5CX_STATUS_OK) {
    return 0;
  }
  return order;
}

/*!
 * @brief Set ranging mode (continuous or autonomous)
 * @param mode VL53L5CX_RANGING_MODE_CONTINUOUS (1) or
 *             VL53L5CX_RANGING_MODE_AUTONOMOUS (3)
 * @return true on success
 */
bool Adafruit_VL53L5CX::setRangingMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_ranging_mode(&_config, mode) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current ranging mode
 * @return VL53L5CX_RANGING_MODE_CONTINUOUS (1) or
 *         VL53L5CX_RANGING_MODE_AUTONOMOUS (3), or 0 on error
 */
uint8_t Adafruit_VL53L5CX::getRangingMode(void) {
  if (!_initialized) {
    return 0;
  }
  uint8_t mode = 0;
  if (vl53l5cx_get_ranging_mode(&_config, &mode) != VL53L5CX_STATUS_OK) {
    return 0;
  }
  return mode;
}

/*!
 * @brief Set power mode (sleep or wakeup)
 * @param mode VL53L5CX_POWER_MODE_SLEEP (0) or
 *             VL53L5CX_POWER_MODE_WAKEUP (1)
 * @return true on success
 */
bool Adafruit_VL53L5CX::setPowerMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  return (vl53l5cx_set_power_mode(&_config, mode) == VL53L5CX_STATUS_OK);
}

/*!
 * @brief Get current power mode
 * @return VL53L5CX_POWER_MODE_SLEEP (0) or
 *         VL53L5CX_POWER_MODE_WAKEUP (1), or 0xFF on error
 */
uint8_t Adafruit_VL53L5CX::getPowerMode(void) {
  if (!_initialized) {
    return 0xFF;
  }
  uint8_t mode = 0;
  if (vl53l5cx_get_power_mode(&_config, &mode) != VL53L5CX_STATUS_OK) {
    return 0xFF;
  }
  return mode;
}

/*!
 * @brief Change the I2C address of the sensor
 * @param new_address New 7-bit I2C address
 * @return true on success
 */
bool Adafruit_VL53L5CX::setAddress(uint8_t new_address) {
  if (!_initialized) {
    return false;
  }
  // ST driver uses 8-bit address internally
  if (vl53l5cx_set_i2c_address(&_config, new_address << 1) !=
      VL53L5CX_STATUS_OK) {
    return false;
  }
  // Update our I2C device to new address
  delete _i2c_dev;
  _i2c_dev = new Adafruit_I2CDevice(new_address, &Wire);
  _config.platform.address = new_address;
  _config.platform.i2c_dev = _i2c_dev;
  return _i2c_dev->begin();
}
