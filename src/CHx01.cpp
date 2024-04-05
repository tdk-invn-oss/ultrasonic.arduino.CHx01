/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
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

#include "CHx01.h"
#include "Arduino.h"
#include "Wire.h"

#include "board/chbsp_chirp.h"
#include <invn/soniclib/ch_rangefinder.h>

// i2c
static bool is_measure_ready = false;

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t int_type) {
  if (int_type == CH_INTERRUPT_TYPE_DATA_RDY) {
    is_measure_ready = true;
  }
}

// CHx01 constructor 
CHx01::CHx01(TwoWire& i2c_ref, uint8_t int1_id, uint8_t int_dir_id,
                   uint8_t rst_id, uint8_t prog_id, bool rst_n=true) {
  chbsp_module_init(i2c_ref, int1_id, int_dir_id, rst_id, prog_id, rst_n);
}

/* Initialize hardware and ICU sensor */
int CHx01::begin() {
  uint8_t rc = 0;
  chbsp_board_init(&chirp_group);
  rc = ch_init(&chirp_device, &chirp_group, 0, fw_init_func);

  if (rc == 0) {
    rc = ch_group_start(&chirp_group);
  }
  if (rc == 0) {
    /* Register callback function to be called when Chirp sensor interrupts */
    ch_io_int_callback_set(&chirp_group, sensor_int_callback);
  }
  return rc;
}

uint16_t CHx01::get_max_samples(void) {
  return ch_get_max_samples(&chirp_device);
};

uint16_t CHx01::get_max_range(void) {
  return ch_samples_to_mm(&chirp_device, ch_get_max_samples(&chirp_device));
};

uint16_t CHx01::get_measure_range(void) {
  uint16_t nb_samples = ch_get_num_samples(&chirp_device);
  return ch_samples_to_mm(&chirp_device, nb_samples);
};

int CHx01::free_run(void) { return free_run(get_max_range()); }

int CHx01::free_run(uint16_t range_mm) {
  return free_run(range_mm, default_odr_ms);
}

int CHx01::algo_config(void)
{
  ch_rangefinder_algo_config_t algo_config;
  algo_config.static_range = 0;
  algo_config.thresh_ptr = NULL;
  return ch_rangefinder_set_algo_config(&chirp_device, &algo_config);
}

int CHx01::free_run(uint16_t range_mm, uint16_t interval_ms) {
  int rc;

  rc = ch_set_max_range(&chirp_device, range_mm);

  if (rc == 0) {
    rc = ch_set_freerun_interval(&chirp_device, interval_ms);
  }
  if (rc == 0) {
    rc = ch_freerun_time_hop_enable(&chirp_device);
  }
  /* Apply GPR configuration */
  if (rc == 0) {
    rc = algo_config();
  }
  if (rc == 0) {
    rc = ch_set_mode(&chirp_device, CH_MODE_FREERUN);
  }
  if (rc == 0) {
    chdrv_int_set_dir_in(&chirp_device);
    chdrv_int_interrupt_enable(&chirp_device);
  }

  return rc;
}

bool CHx01::data_ready(void) { return is_measure_ready; }

void CHx01::clear_data_ready(void) { is_measure_ready = false; }

uint16_t CHx01::part_number(void) {
  return ch_get_part_number(&chirp_device);
}

uint32_t CHx01::frequency(void) { return ch_get_frequency(&chirp_device); }

uint16_t CHx01::bandwidth(void) { return ch_get_bandwidth(&chirp_device); }

uint16_t CHx01::rtc_cal(void) {
  return ch_get_rtc_cal_result(&chirp_device);
}

uint16_t CHx01::rtc_cal_pulse_length(void) {
  return ch_get_rtc_cal_pulselength(&chirp_device);
}

float CHx01::cpu_freq(void) {
  return ch_get_cpu_frequency(&chirp_device) / 1000000.0f;
}
const char *CHx01::fw_version(void) {
  return ch_get_fw_version_string(&chirp_device);
}

void CHx01::print_informations(void) {
  Serial.println("Sensor informations");
  Serial.print("    Type: ");
  Serial.println(part_number());
  Serial.print("    Operating Frequency (Hz): ");
  Serial.println(frequency());
  Serial.print("    Bandwidth (Hz): ");
  Serial.println(bandwidth());
  Serial.print("    RTC Cal (lsb @ ms): ");
  Serial.print(rtc_cal());
  Serial.print("@");
  Serial.println(rtc_cal_pulse_length());
  Serial.print("    CPU Freq (MHz): ");
  Serial.println(cpu_freq());
  Serial.print("    max range (mm): ");
  Serial.println(get_max_range());
  Serial.print("    Firmware: ");
  Serial.println(fw_version());
  Serial.print("    Nb samples: ");
  Serial.println(ch_get_max_samples(&chirp_device));
}

void CHx01::print_configuration(void) {
  Serial.println("Sensor configuration");
  Serial.print("    measure range (mm): ");
  Serial.println(get_measure_range());
}

float CHx01::get_range(void) {
  float range_mm = 0.0;
  uint32_t range_q5;

  if (data_ready()) {
    range_q5 = ch_get_range(&chirp_device, CH_RANGE_ECHO_ONE_WAY);
    if (range_q5 != CH_NO_TARGET) {
      /* Display single detected target (if any) */
      range_mm = range_q5 / 32.0;
    } else {
      /* No target detected: range set to 0 */
      range_mm = 0.0;
    }
    clear_data_ready();
  }
  return range_mm;
}