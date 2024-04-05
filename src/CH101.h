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

#ifndef CH101_H
#define CH101_H

#include "Arduino.h"
#include "Wire.h"
#include "CHx01.h"
#include <stdint.h>
#include <string.h>


extern "C" {
#include "invn/soniclib/sensor_fw/ch101/ch101_gpr.h"
}

#include "invn/soniclib/soniclib.h"

class CH101 : public CHx01 {
public:
  /*!
   * @brief Class constructor.
   * @param i2c_ref Reference of the Wire to be used
   * @param int1_id ID of the interrupt 1 pin
   * @param int_dir_id ID of the interrupt direction pin
   * @param rst_id ID of the interrupt reset pin
   * @param prog_id ID of the interrupt prog pin
   */
  CH101(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id,uint8_t reset_id,
           uint8_t prog_id, bool rst_n=true) : CHx01(i2c_ref,int1_id,int_dir_id,reset_id,prog_id, rst_n)
          {fw_init_func = ch101_gpr_init;};

  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = CH101_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[CH101_MAX_NUM_SAMPLES], uint16_t& nb_samples);
};

#endif // CH101_H
