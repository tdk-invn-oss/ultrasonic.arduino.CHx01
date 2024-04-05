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

// Hardware config : INT1 on pin 2, INT_DIR on 8, Reset on 9 (not inverted) and Prog on 10
CH101 CHx01(Wire, 2, 8, 9 , 10, false);

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("CH101 General purpose detection");

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
  // Start CHx01 in free run mode
  ret = CHx01.free_run();
  if (ret != 0) {
    Serial.print("CH101 free run failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    CHx01.print_configuration();
  }
}

void loop() {
  float range;

  /* Wait for new measure done */
  if (CHx01.data_ready()) {
    /* Get range to target computed by sensor */
    range = CHx01.get_range();
    Serial.print("Range(mm):");
    Serial.println(range);
  }
}
