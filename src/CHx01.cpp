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
#include <math.h>

#include "board/chbsp_chirp.h"

#define RTC_CAL_PULSE_MS (100)

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t int_type) {
  if (int_type == CH_INTERRUPT_TYPE_DATA_RDY) {
    ((CHx01*)grp_ptr)->get_device(dev_num)->set_data_ready();
  }
}

// CHx01 constructor: the group has an existing sensor object
CHx01::CHx01(CHx01_dev* dev, int rst_id, bool rst_n)
{
  /* Initialize group descriptor */
  ch_group_init(this, 1, 1, RTC_CAL_PULSE_MS);

  device[0] = dev;
  chbsp_module_init(rst_id, rst_n);
}

CHx01::CHx01(CHx01_dev* dev0, CHx01_dev* dev1, int rst_id, bool rst_n)
{
  /* Initialize group descriptor */
  ch_group_init(this, 2, 1, RTC_CAL_PULSE_MS);

  device[0] = dev0;
  device[1] = dev1;
  chbsp_module_init(rst_id, rst_n);
}

CHx01::CHx01(CHx01_dev& dev0, CHx01_dev& dev1, int rst_id, bool rst_n)
{
  /* Initialize group descriptor */
  ch_group_init(this, 2, 1, RTC_CAL_PULSE_MS);

  device[0] = &dev0;
  device[1] = &dev1;
  chbsp_module_init(rst_id, rst_n);
}

CHx01_dev* CHx01::get_device(int id)
{
  if (id < num_ports)
  {
    return (CHx01_dev*)device[id];
  }
  return NULL;
}

/* Initialize hardware and ICU sensor */
int CHx01::begin() {
  uint8_t rc = 0;

  board_init(this);

  for(int i= 0; i < num_ports; i++)
  {
    rc |= get_device(i)->begin((ch_group_t*)this,i);
  }

  if (rc == 0) {
    rc = ch_group_start(this);
  }
  if (rc == 0) {
    /* Register callback function to be called when Chirp sensor interrupts */
    ch_io_int_callback_set(this, sensor_int_callback);
  }
  return rc;
}

uint16_t CHx01::get_max_samples(int sensor_id) {
  return get_device(sensor_id)->get_max_samples();
};

uint16_t CHx01::get_max_range(int sensor_id) {
  return get_device(sensor_id)->get_max_range();
};

uint16_t CHx01::get_measure_range(int sensor_id) {
  return get_device(sensor_id)->get_measure_range();
};

int CHx01::free_run(void) { return get_device(0)->free_run(); }

int CHx01::free_run(uint16_t range_mm) {
  return get_device(0)->free_run(range_mm);
}

int CHx01::algo_config(int sensor_id)
{
  return get_device(sensor_id)->algo_config();
}

int CHx01::free_run(uint16_t range_mm, uint16_t interval_ms) {
  return get_device(0)->free_run(range_mm,interval_ms);
}

int CHx01::start_trigger(uint16_t range_mm) {
  int rc = 0;
  rc |= get_device(0)->start_trigger(range_mm,CH_MODE_TRIGGERED_TX_RX);
  rc |= get_device(1)->start_trigger(range_mm,CH_MODE_TRIGGERED_RX_ONLY);
  return rc;
}

void CHx01::trig(void) {
  return ch_group_trigger(this);
}

bool CHx01::data_ready(int sensor_id) {
  return get_device(sensor_id)->data_ready();
}

uint16_t CHx01::part_number(int sensor_id) {
  return get_device(sensor_id)->part_number();
}

uint32_t CHx01::frequency(int sensor_id) {
  return get_device(sensor_id)->frequency();
}

uint16_t CHx01::bandwidth(int sensor_id) {
  return get_device(sensor_id)->bandwidth();
}

uint16_t CHx01::rtc_cal(int sensor_id) {
  return get_device(sensor_id)->rtc_cal();
}

uint16_t CHx01::rtc_cal_pulse_length(int sensor_id) {
  return get_device(sensor_id)->rtc_cal_pulse_length();
}

float CHx01::cpu_freq(int sensor_id) {
  return get_device(sensor_id)->cpu_freq();
}
const char *CHx01::fw_version(int sensor_id) {
  return get_device(sensor_id)->fw_version();
}

void CHx01::print_informations(void) {
  for(int i = 0; i < num_ports; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(":");
    get_device(i)->print_informations();
  }
}

void CHx01::print_configuration(void) {
  for(int i = 0; i < num_ports; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(":");
    get_device(i)->print_configuration();
  }
}

float CHx01::get_range(int sensor_id) {
  return get_device(sensor_id)->get_range();
}

int CHx01::triangulate(const float distance_between_sensors_mm, float& x, float& y, float offset)
{
  int rc = 0;
  float range0_mm = get_range(0);
  float range1_mm = get_range(1);
  float diff_mm;

  if ((range0_mm == 0)||(range1_mm == 0)||(range1_mm<=range0_mm))
  {
    /* One of the sensor losts the target */
    return -1;
  }
  /* Remove transmit distance to the 2nd sensor distance */
  range1_mm -= range0_mm + offset;

  diff_mm = (range0_mm > range1_mm) ? (range0_mm - range1_mm) : (range1_mm - range0_mm);
  if(diff_mm > distance_between_sensors_mm)
  {
    /* This is not supposed to happen geometrically */
    return -2;
  }
  x =  (range0_mm*range0_mm - range1_mm*range1_mm) / (2*distance_between_sensors_mm);

  y = distance_between_sensors_mm/2 +x;
  y = (range0_mm-y)*(range0_mm + y);
  if(y>0)
  {
    y = sqrt(y);
  } else {
    y = 0;
    return -3;
  }
  return 0;
}