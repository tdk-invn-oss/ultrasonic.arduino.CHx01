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
  return get_iq_data(0,iq_data,nb_samples);
}

uint8_t CH201::get_iq_data(int sensor_id, ch_iq_sample_t (&iq_data)[CH201_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  CH201_dev* device = (CH201_dev*)get_device(sensor_id);
  return device->get_iq_data(iq_data,nb_samples);
}

int CH201::algo_config(int sensor_id)
{
  return get_device(sensor_id)->algo_config();
}

uint16_t CH201::get_max_range(int sensor_id) {
  return get_device(sensor_id)->get_max_range();
}
