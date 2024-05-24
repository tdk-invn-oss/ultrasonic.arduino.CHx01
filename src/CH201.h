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

#ifndef CH201_H
#define CH201_H

#include "Arduino.h"
#include "Wire.h"
#include "CHx01.h"
#include "CH201_dev.h"
#include <stdint.h>
#include <string.h>


extern "C" {
#include "invn/soniclib/sensor_fw/ch201/ch201_gprmt.h"
}

#include "invn/soniclib/soniclib.h"

class CH201 : public CHx01 {
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
  CH201(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id, uint8_t reset_id,
           uint8_t prog_id, bool rst_n=true) : CHx01(new CH201_dev(i2c_ref, CHIRP_DEVICE0_I2C_ADDR, int1_id, int_dir_id, prog_id),reset_id, rst_n) {};

  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = CH201_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[CH201_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  
  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = CH201_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(int sensor_id, ch_iq_sample_t (&iq_data)[CH201_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  
  /*!
   * @brief Configure embedded algo.
   * @return 0 if successful, 1 if error
   */
  int algo_config(int sensor_id=0);
  /*!
   * @brief Get the sensor's max range.
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  uint16_t get_max_range(int sensor_id=0);
};
#endif // CH201_H
