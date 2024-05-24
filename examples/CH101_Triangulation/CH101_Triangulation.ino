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

#include "CH101.h"

#define DISTANCE_BETWEEN_SENSORS_MM 60 
// Hardware config : 2 sensors INT1 on pin 2 & pin 3, INT_DIR on 8, Reset on 9 (not inverted) and Prog on 10 & 11
CH101_dev dev0(Wire, CHIRP_DEVICE0_I2C_ADDR, 2, 8, 10);
CH101_dev dev1(Wire, CHIRP_DEVICE1_I2C_ADDR, 3, 8, 11);
CHx01 CHx01(dev0,dev1, 9, false);

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("CH101 Triangulation");

  // Initializing the CHx01 device
  ret = CHx01.begin();
  if (ret != 0) {
    Serial.print("CH101 initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    CHx01.print_informations();
  }
  // Start CHx01 in trigger mode (500mm)
  ret = CHx01.start_trigger(500);
  if (ret != 0) {
    Serial.print("CH101 trig start failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    CHx01.print_configuration();
  }
  CHx01.trig();
}

void loop() {
  float range;
  /* Wait for new measure done */
  if (CHx01.data_ready(0)&&CHx01.data_ready(1)) {
    float x,y;
    if(CHx01.triangulate(DISTANCE_BETWEEN_SENSORS_MM,x,y)==0)
    {
      Serial.print("X:");
      Serial.println(x);
      Serial.print(" Y:");
      Serial.println(y);
    }
    /* Trig next measurement */
    CHx01.trig();
    delay(100);
  }
}
