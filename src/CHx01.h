/*
 *
 * Copyright (c) 2024 by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef CHx01_H
#define CHx01_H

#include "Arduino.h"
#include <stdint.h>
#include <string.h>
#include "CHx01_dev.h"

#include "invn/soniclib/soniclib.h"

#define CHIRP_DEVICE0_I2C_ADDR 0x2D
#define CHIRP_DEVICE1_I2C_ADDR 0x2C

class CHx01 : public ch_group_t {
public:
  /*!
   * @brief Class constructor for a single device in the group.
   * @param dev CH101_dev or CH201_dev object pointer to be added to the group
   * @param rst_id ID of the interrupt reset pin
   * @param rst_n If true the reset signal is active LOW (default)
   */
  CHx01(CHx01_dev* dev, int rst_id, bool rst_n=true);
  /*!
   * @brief Class constructor for a 2 devices group.
   * @param dev0 CH101_dev or CH201_dev object pointer to be added to the group
   * @param dev1 CH101_dev or CH201_dev object pointer to be added to the group
   * @param rst_id ID of the interrupt reset pin
   * @param rst_n If true the reset signal is active LOW (default)
   */
  CHx01(CHx01_dev* dev0, CHx01_dev* dev1, int rst_id, bool rst_n=true);
  /*!
   * @brief Class constructor for a 2 devices group.
   * @param dev0 CH101_dev or CH201_dev object to be added to the group
   * @param dev1 CH101_dev or CH201_dev object to be added to the group
   * @param rst_id ID of the interrupt reset pin
   * @param rst_n If true the reset signal is active LOW (default)
   */
  CHx01(CHx01_dev& dev0, CHx01_dev& dev1, int rst_id, bool rst_n=true);
  /*!
   * @brief Configure the sensor group
   * @return 0 if successful
   */
   int begin();
  /*!
   * @brief Get a device from the group
   * @param id Id of the device to get
   * @return A pointer to the device defined by id
   */
   CHx01_dev* get_device(int id);
  /*!
   * @brief Get the sensor part number.
   * @param id Id of the device in the group
   * @return Sensor part number as an integer
   */
  uint16_t part_number(int sensor_id=0);
  /*!
   * @brief Get sensor's operating frequency.
   * @param id Id of the device in the group
   * @return Acoustic operating frequency, in Hz
   */
  uint32_t frequency(int sensor_id=0);
  /*!
   * @brief Get sensor's bandwidth.
   * @param id Id of the device in the group
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t bandwidth(int sensor_id=0);
  /*!
   * @brief Get the real-time clock calibration value.
   * @param id Id of the device in the group
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t rtc_cal(int sensor_id=0);
  /*!
   * @brief Get the real-time clock calibration pulse length.
   * @param id Id of the device in the group
   * @return RTC pulse length, in ms
   */
  uint16_t rtc_cal_pulse_length(int sensor_id=0);
  /*!
   * @brief Get the sensor CPU frequency, in Hz.
   * @param id Id of the device in the group
   * @return sensor CPU frequency, in Hz
   */
  float cpu_freq(int sensor_id=0);
  /*!
   * @brief Get the firmware version description string.
   * @param id Id of the device in the group
   * @return Pointer to character string describing sensor firmware version
   */
  const char *fw_version(int sensor_id=0);
  /*!
   * @brief Check if a measure is available.
   * @param id Id of the device in the group
   * @return True if a measure is available, false otherwise
   */
  bool data_ready(int sensor_id=0);
  /*!
   * @brief Get the sensor's max number of samples.
   * @param id Id of the device in the group
   * @return Sensor's max number of sample
   * This value does not depend on sensor current configuration
   */
  uint16_t get_max_samples(int sensor_id=0);
  /*!
   * @brief Get the sensor's max range.
   * @param id Id of the device in the group
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  virtual uint16_t get_max_range(int sensor_id=0);
  /*!
   * @brief Get the sensor's range configured for measures.
   * @param id Id of the device in the group
   * @return Sensor's measure range in mm
   * This value depends on sensor configured range
   */
  uint16_t get_measure_range(int sensor_id=0);
    /*!
   * @brief Configure embedded algo.
   * @param id Id of the device in the group
   * @return 0 if successful, 1 if error
   */
  virtual int algo_config(int sensor_id=0);
  /*!
   * @brief Starts sensor in free running mode, with maximum range and default ODR.
   * @return 0 in case of success
   */
  int free_run(void);
  /*!
   * @brief Starts sensor 0 in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor 0 in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm, uint16_t interval_ms);
  /*!
   * @brief Starts sensor in trigger mode, with provided max range.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int start_trigger(uint16_t range_mm);
  /*!
   * @brief Trig sensor measurement.
   */
  void trig(void);
  /*!
   * @brief Displays sensor information.
   */
  void print_informations(void);
  /*!
   * @brief Displays sensor configuration.
   */
  void print_configuration(void);
  /*!
   * @brief Get the measured range from a sensor.
   * @param id Id of the device in the group
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
  float get_range(int sensor_id=0);
  /*!
   * @brief Triangulate an object using 2 sensors.
   * @param distance_between_sensors_mm The distance between the 2 sensors in mm
   * @param x Output object abscissa
   * @param y Output object ordinate
   * @param offset 2nd sensor offset in mm 
   * @return 0 in case of success
   */
  int triangulate(const float distance_between_sensors_mm, float& x, float& y, float offset=0);
};

#endif // CHx01_H
