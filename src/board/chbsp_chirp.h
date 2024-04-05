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

#include <invn/soniclib/chirp_bsp.h>
#include "Wire.h"

#ifndef CHBSP_CHIRP_H
#define CHBSP_CHIRP_H

#define UNUSED_PIN (0xFF)
#define DEFAULT_I2C_CLOCK 400000
#define MAX_I2C_CLOCK 400000

void chbsp_module_init(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id, uint8_t rst_id, uint8_t prog_id, bool rst_n);


#endif