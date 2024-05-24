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
#include "Wire.h"
#include <stdint.h>
#include "chbsp_chirp.h"

// Chirp SonicLib API definitions
#include <invn/soniclib/chirp_bsp.h>
#include <invn/soniclib/soniclib.h>

#define ARDUINO_I2C_BUFFER_LENGTH 32

CHx01* sensor_group_ptr = NULL;
static int rst_pin_id;
static int reset_n;
static void int1_0_handler(void);
static void int1_1_handler(void);
const chx01_dev_irq_handler *irq_handlers[2] = {int1_0_handler, int1_1_handler};


extern "C" int chbsp_i2c_init(void)
{
  Wire.begin();
  Wire.setClock(DEFAULT_I2C_CLOCK);
  return 0;
}

void chbsp_module_init(int rst_id, bool rst_n)
{
  rst_pin_id = rst_id;
  reset_n = rst_n;
}

extern "C" void chbsp_i2c_reset(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->i2c_reset();
}

static void int1_0_handler(void)
{
  if(sensor_group_ptr != NULL)
  {
    ch_interrupt(sensor_group_ptr, 0);
    // Data ready handler disables interrupt, make it ready to fire again !
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(sensor_group_ptr, 0));
  }
}
static void int1_1_handler(void)
{
  if(sensor_group_ptr != NULL)
  {
    ch_interrupt(sensor_group_ptr, 1);
    // Data ready handler disables interrupt, make it ready to fire again !
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(sensor_group_ptr, 1));
  }
}

void board_init(CHx01 *grp_ptr)
{
  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;

  /* Configure Reset pin as output, set to 1 (Reset enabled, inverted to RST_N on the board) */
  pinMode(rst_pin_id, OUTPUT);
  digitalWrite(rst_pin_id,reset_n?LOW:HIGH);
}

extern "C" void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->set_int1_dir(CHX01_INT_DIR_OUT);

}

extern "C" void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->set_int1_dir(CHX01_INT_DIR_IN);
}

extern "C" void chbsp_int1_clear(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->set_int1(LOW);
}

extern "C" void chbsp_int1_set(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->set_int1(HIGH);
}

extern "C" void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;

  if(device->io_index < CHIRP_MAX_NUM_SENSORS)
  {
    device->enableInterrupt(irq_handlers[device->io_index]);
  }
}

extern "C" void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  device->disableInterrupt();
}

extern "C" uint8_t chbsp_i2c_get_info(ch_group_t* grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr)
{
  CHx01* group_ptr = (CHx01*)grp_ptr;
  CHx01_dev* device = group_ptr->get_device(io_index);
  if(device != NULL)
  {
    info_ptr->address   = device->get_i2c_addr();
    /* All sensors on same bus */
    info_ptr->bus_num   = 0;
    info_ptr->drv_flags = 0; /* no special I2C handling by SonicLib driver is needed */
    return 0;
  }
  return -1;
}

extern "C" int chbsp_i2c_write(ch_dev_t *dev_ptr, const uint8_t *data, uint16_t num_bytes)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  TwoWire* i2c = device->get_i2c();
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

extern "C" int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  TwoWire* i2c = device->get_i2c();
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

extern "C" int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  TwoWire* i2c = device->get_i2c();
  
  i2c->beginTransmission(ch_get_i2c_address(dev_ptr));
  i2c->write((uint8_t)mem_addr);
  /* TODO: the doc says we should send num_bytes here */
  for(uint8_t i = 0; i < num_bytes; i++) {
    i2c->write(data[i]);
  }
  i2c->endTransmission();
  return 0;
}

extern "C" int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
  CHx01_dev* device = (CHx01_dev*)dev_ptr;
  TwoWire* i2c = device->get_i2c();
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

extern "C" void chbsp_reset_assert(void)
{
  /* CHX01_RST_N = 0 */
  digitalWrite(rst_pin_id,reset_n ? LOW : HIGH);
}

extern "C" void chbsp_reset_release(void)
{
  /* CHX01_RST_N = 1 */
  digitalWrite(rst_pin_id,reset_n ? HIGH : LOW);
}

extern "C" void chbsp_program_enable(ch_dev_t *dev_ptr)
{
  ((CHx01_dev*)dev_ptr)->set_prog(HIGH);
}

extern "C" void chbsp_program_disable(ch_dev_t *dev_ptr)
{
  ((CHx01_dev*)dev_ptr)->set_prog(LOW);
}

extern "C" void chbsp_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

extern "C" void chbsp_delay_ms(uint32_t ms)
{
  delay(ms);
}

extern "C" void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_set_int1_dir_out(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_set_int1_dir_in(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_clear(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_clear(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_set(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_set(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_interrupt_disable(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" uint32_t chbsp_timestamp_ms(void)
{
  return (uint32_t) millis();
}