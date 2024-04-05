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
 
 #include "CH201.h"

uint8_t CH201::get_iq_data(ch_iq_sample_t (&iq_data)[CH201_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  nb_samples  = ch_get_num_samples(&chirp_device);

  /* Reading I/Q data in normal, blocking mode */
  uint8_t err = ch_get_iq_data(&chirp_device, iq_data, 0, nb_samples,
                        CH_IO_MODE_BLOCK);
  clear_data_ready();

  return err;
}

int CH201::algo_config(void)
{
  ch_rangefinder_algo_config_t algo_config;
  algo_config.static_range = 0;
  algo_config.thresh_ptr = &chirp_detect_thresholds;
  return ch_rangefinder_set_algo_config(&chirp_device, &algo_config);
}


uint16_t CH201::get_max_range(void) {
  uint16_t sensor_max_range = CHx01::get_max_range();
  return(sensor_max_range > max_range) ? max_range : sensor_max_range ;
};
