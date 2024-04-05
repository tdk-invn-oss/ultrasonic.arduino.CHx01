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
   */
  CH201(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id, uint8_t reset_id,
           uint8_t prog_id, bool rst_n=true) : CHx01(i2c_ref,int1_id,int_dir_id,reset_id,prog_id,rst_n)
          {fw_init_func = ch201_gprmt_init;};

  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = CH201_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[CH201_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  
  /*!
   * @brief Configure embedded algo.
   * @return 0 if successful, 1 if error
   */
  int algo_config(void);
  /*!
   * @brief Get the sensor's max range.
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  uint16_t get_max_range(void);
protected:
/* Detection threshold settings - only for sensors using multi-threshold firmware
 *   (e.g. ch101_gprmt or ch201_gprmt).
 *   Each threshold entry includes the starting sample number & threshold level.
 */
  ch_thresholds_t chirp_detect_thresholds = {.threshold = {
   {CHIRP_THRESH_0_START, CHIRP_THRESH_0_LEVEL}, /* threshold 0 */
   {CHIRP_THRESH_1_START, CHIRP_THRESH_1_LEVEL}, /* threshold 1 */
   {CHIRP_THRESH_2_START, CHIRP_THRESH_2_LEVEL}, /* threshold 2 */
   {CHIRP_THRESH_3_START, CHIRP_THRESH_3_LEVEL}, /* threshold 3 */
   {CHIRP_THRESH_4_START, CHIRP_THRESH_4_LEVEL}, /* threshold 4 */
   {CHIRP_THRESH_5_START, CHIRP_THRESH_5_LEVEL}, /* threshold 5 */
}};
};
  const uint16_t max_range = 2400;
#endif // CH201_H
