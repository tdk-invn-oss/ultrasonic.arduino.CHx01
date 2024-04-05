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

#include "Arduino.h"
#include <stdint.h>
#include "chbsp_chirp.h"

// Chirp SonicLib API definitions
#include <invn/soniclib/chirp_bsp.h>
#include <invn/soniclib/soniclib.h>

#define CHX01_INT_DIR_OUT (1)
#define CHX01_INT_DIR_IN  (0)

#define CHBSP_RTC_CAL_PULSE_MS       (100)

#define ARDUINO_I2C_BUFFER_LENGTH 32

static TwoWire* i2c = NULL;
static const uint8_t chx01_i2c_addrs[] = {0x2D, 0x2C, 0x2B, 0x2A};

static uint8_t int1_pin_id;
static uint8_t int_dir_pin_id;
static uint8_t rst_pin_id;
static uint8_t prog_pin_id;
static bool reset_n = true;
static ch_group_t *sensor_group_ptr = NULL;

static bool int1_attached = false;
static bool int1_in_dir = false;

static void int1_handler(void);

static void sensors_pin_init(ch_dev_t *dev_ptr)
{
  /* Configure INT pins as input */
  chbsp_set_int1_dir_in(dev_ptr);

  /* Enable pull-ups on the INT pins */
  pinMode(int1_pin_id, INPUT_PULLUP);

  /* Configure INT DIR pin as output */
  if(int_dir_pin_id != UNUSED_PIN)
  {
    pinMode(int_dir_pin_id, OUTPUT);
    digitalWrite(int_dir_pin_id,CHX01_INT_DIR_IN);
  }
  /* TODO: manage inversion or not on board */
  /* Configure Reset pin as output, set to 1 (Reset enabled, inverted to RST_N on the board) */
  pinMode(rst_pin_id, OUTPUT);
  digitalWrite(rst_pin_id,HIGH);

  /* Configure Prog pin as output, set to 0 */
  pinMode(prog_pin_id, OUTPUT);
  digitalWrite(prog_pin_id,LOW);

}

int chbsp_i2c_init(void)
{
  i2c->begin();
  i2c->setClock(DEFAULT_I2C_CLOCK);
  return 0;
}

void chbsp_i2c_reset(ch_dev_t *dev_ptr)
{
  /* Not sure Arduino support I2C bus reset... */
  (void)dev_ptr;
  i2c->begin();
  i2c->setClock(DEFAULT_I2C_CLOCK);
}

static void int1_handler(void)
{
  if(sensor_group_ptr != NULL)
  {
    ch_interrupt(sensor_group_ptr, 0);
    // Data ready handler disables interrupt, make it ready to fire again !
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(sensor_group_ptr, 0));
  }
}

void chbsp_module_init(TwoWire &i2c_ref, uint8_t int1_id, uint8_t int_dir_id, uint8_t rst_id, uint8_t prog_id, bool rst_n)
{
  i2c = &i2c_ref;
  int1_pin_id = int1_id;
  int_dir_pin_id = int_dir_id;
  rst_pin_id = rst_id;
  prog_pin_id = prog_id;
  reset_n = rst_n;
}

void chbsp_board_init(ch_group_t *grp_ptr)
{
  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;

  /* Initialize group descriptor */
  grp_ptr->num_ports = 1;
  grp_ptr->num_buses = 1;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
  grp_ptr->disco_hook = NULL;

  chbsp_i2c_init();

  sensors_pin_init(grp_ptr->device[0]);
}

void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)
{
  if(int1_in_dir)
  {
    if(int_dir_pin_id != UNUSED_PIN)
    {
      digitalWrite(int_dir_pin_id,CHX01_INT_DIR_OUT);
    }
    pinMode(int1_pin_id,OUTPUT);
    int1_in_dir = false;
  }
}

void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)
{
  if(!int1_in_dir)
  {
    if(int_dir_pin_id != UNUSED_PIN)
    {
      digitalWrite(int_dir_pin_id,CHX01_INT_DIR_IN);
    }
    pinMode(int1_pin_id,INPUT_PULLUP);
    int1_in_dir = true;
  }
}

void chbsp_int1_clear(ch_dev_t *dev_ptr)
{
    digitalWrite(int1_pin_id,LOW);
}

void chbsp_int1_set(ch_dev_t *dev_ptr)
{
    digitalWrite(int1_pin_id,HIGH);
}

void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr)
{
  if (!int1_attached) {
    chbsp_set_int1_dir_in(dev_ptr);

    attachInterrupt(digitalPinToInterrupt(int1_pin_id),int1_handler,FALLING);
    int1_attached = true;
  }
}

void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr)
{
  if (int1_attached) {
    detachInterrupt(digitalPinToInterrupt(int1_pin_id));
    int1_attached = false;
  }
}

uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) * grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr)
{

  info_ptr->address   = chx01_i2c_addrs[io_index];
  /* All sensors on same bus */
  info_ptr->bus_num   = 0;
  info_ptr->drv_flags = 0; /* no special I2C handling by SonicLib driver is needed */
  return 0;
}

int chbsp_i2c_write(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  uint16_t offset = 0;
  while(offset < num_bytes)
  {
    i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
    uint16_t length = ((num_bytes - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (num_bytes - offset) ;
    i2c->write(&data[offset],length);
    offset += length;
    i2c->endTransmission((offset == num_bytes));
  }

  return 0;
}

int chbsp_i2c_read(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  uint16_t offset = 0;
  
  while(offset < num_bytes)
  {
    uint16_t rx_bytes = 0;
    i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
    uint16_t length = ((num_bytes - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (num_bytes - offset) ;
    rx_bytes = i2c->requestFrom((int)ch_get_i2c_address(dev_ptr), length);
    if (rx_bytes == length) {
       for(uint8_t i = 0; i < length; i++) {
      data[offset+i] = i2c->read();
      }
      offset += length;
      i2c->endTransmission((offset == num_bytes));
    } else {
      i2c->endTransmission((offset == num_bytes));
    }
  }
  if(offset == num_bytes)
  {
    return 0;
  } else {
    return -1;
  }
}

int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
  i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
  i2c->write((uint8_t)mem_addr);
  /* TODO: the doc says we should send num_bytes here */
  for(uint8_t i = 0; i < num_bytes; i++) {
    i2c->write(data[i]);
  }
  i2c->endTransmission();
  return 0;
}

int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
  uint16_t rx_bytes = 0;
  i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
  i2c->write((uint8_t)mem_addr);
  /* TODO: the doc says we should send num_bytes here */
  i2c->endTransmission(false);
  
  i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
  rx_bytes = i2c->requestFrom((int)ch_get_i2c_address(dev_ptr), num_bytes);
  if (rx_bytes == num_bytes) {
    for(uint8_t i = 0; i < num_bytes; i++) {
      data[i] = i2c->read();
    }
    i2c->endTransmission();
    return 0;
  } else {
    return -1;
  }
}


void chbsp_reset_assert(void)
{
  /* CHX01_RST_N = 0 */
  digitalWrite(rst_pin_id,reset_n ? LOW : HIGH);
}

void chbsp_reset_release(void)
{
  /* CHX01_RST_N = 1 */
  digitalWrite(rst_pin_id,reset_n ? HIGH : LOW);
}

void chbsp_program_enable(ch_dev_t *dev_ptr)
{
  digitalWrite(prog_pin_id,HIGH);
}

void chbsp_program_disable(ch_dev_t *dev_ptr)
{
  digitalWrite(prog_pin_id,LOW);
}

void chbsp_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

void chbsp_delay_ms(uint32_t ms)
{
  delay(ms);
}

void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr)
{
  chbsp_set_int1_dir_out(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr)
{
  chbsp_set_int1_dir_in(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_int1_clear(ch_group_t *grp_ptr)
{
  chbsp_int1_clear(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_int1_set(ch_group_t *grp_ptr)
{
  chbsp_int1_set(ch_get_dev_ptr(grp_ptr, 0));
}
void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr)
{
  chbsp_int1_interrupt_enable(ch_get_dev_ptr(grp_ptr, 0));
}
void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr)
{
  chbsp_int1_interrupt_disable(ch_get_dev_ptr(grp_ptr, 0));
}

uint32_t chbsp_timestamp_ms(void)
{
  return (uint32_t) millis();
}