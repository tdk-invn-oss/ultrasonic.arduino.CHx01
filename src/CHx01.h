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
#include "Wire.h"
#include <stdint.h>
#include <string.h>

#include "invn/soniclib/soniclib.h"

/* Target detection thresholds (if supported)
 * These definitions set the sensor's target detection thresholds, only for 
 * sensors using multi-threshold firmware (e.g. ch101_gprmt or ch201_gprmt).
 *
 * These thresholds specify how large a signal must be received, and at what 
 * point in the measurement, for the sensor to indicate that a target was 
 * detected and calculate the range.
 *
 * Each threshold consists of a starting sample number within the measurement 
 * and the corresponding amplitude value that must be reached.  A threshold 
 * extends until the starting sample number of the next threshold, if any.
 */
#define CHIRP_THRESH_0_START (0) /* threshold 0 - must start at zero */
#define CHIRP_THRESH_0_LEVEL (5000)

#define CHIRP_THRESH_1_START (26) /* threshold 1 */
#define CHIRP_THRESH_1_LEVEL (2000)

#define CHIRP_THRESH_2_START (39) /* threshold 2 */
#define CHIRP_THRESH_2_LEVEL (800)

#define CHIRP_THRESH_3_START (56) /* threshold 3 */
#define CHIRP_THRESH_3_LEVEL (400)

#define CHIRP_THRESH_4_START (79) /* threshold 4 */
#define CHIRP_THRESH_4_LEVEL (250)

#define CHIRP_THRESH_5_START (89) /* threshold 5 */
#define CHIRP_THRESH_5_LEVEL (175)

class CHx01 {
public:
  /*!
   * @brief Class constructor.
   * @param i2c_ref Reference of the Wire to be used
   * @param int1_id ID of the interrupt 1 pin
   * @param int_dir_id ID of the interrupt direction pin
   * @param rst_id ID of the interrupt reset pin
   * @param prog_id ID of the interrupt prog pin
   * @param rst_n If true the reset signal is active LOW (default)
   */
  CHx01(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id,
           uint8_t reset_id, uint8_t prog_id, bool rst_n);
  /*!
   * @brief Configure the sensor
   * @return 0 if successful
   */
   int begin();
  /*!
   * @brief Get the sensor part number.
   * @return Sensor part number as an integer
   */
  uint16_t part_number(void);

  /*!
   * @brief Get sensor's operating frequency.
   * @return Acoustic operating frequency, in Hz
   */
  uint32_t frequency(void);
  /*!
   * @brief Get sensor's bandwidth.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t bandwidth(void);
  /*!
   * @brief Get the real-time clock calibration value.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t rtc_cal(void);
  /*!
   * @brief Get the real-time clock calibration pulse length.
   * @return RTC pulse length, in ms
   */
  uint16_t rtc_cal_pulse_length(void);
  /*!
   * @brief Get the sensor CPU frequency, in Hz.
   * @return sensor CPU frequency, in Hz
   */
  float cpu_freq(void);
  /*!
   * @brief Get the firmware version description string.
   * @return Pointer to character string describing sensor firmware version
   */
  const char *fw_version(void);
  /*!
   * @brief Check if a measure is available.
   * @return True if a measure is available, false otherwise
   */
  bool data_ready(void);
  /*!
   * @brief Get the sensor's max number of samples.
   * @return Sensor's max number of sample
   * This value does not depend on sensor current configuration
   */
  uint16_t get_max_samples(void);
  /*!
   * @brief Get the sensor's max range.
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  virtual uint16_t get_max_range(void);
  /*!
   * @brief Get the sensor's range configured for measures.
   * @return Sensor's measure range in mm
   * This value depends on sensor configured range
   */
  uint16_t get_measure_range(void);
    /*!
   * @brief Configure embedded algo.
   * @return 0 if successful, 1 if error
   */
  virtual int algo_config(void);
  /*!
   * @brief Starts sensor in free running mode, with maximum range and default ODR.
   * @return 0 in case of success
   */
  int free_run();
  /*!
   * @brief Starts sensor in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm, uint16_t interval_ms);
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
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
   float get_range(void);
protected:
  /* Descriptor structure for group of sensors */
  ch_group_t chirp_group;
  ch_dev_t chirp_device;
  ch_fw_init_func_t fw_init_func;
  static const uint32_t default_odr_ms = 100; // 1 measure each 100 ms
  void clear_data_ready();
};

#endif // CHx01_H
