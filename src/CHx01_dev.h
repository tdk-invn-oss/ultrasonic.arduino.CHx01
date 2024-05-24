
#ifndef CHx01_DEV_H
#define CHx01_DEV_H

#include "Arduino.h"
#include "Wire.h"
#include <stdint.h>
#include <string.h>

#include "invn/soniclib/soniclib.h"

#define UNUSED_PIN (0xFF)
#define DEFAULT_I2C_CLOCK 400000

#define CHX01_INT_DIR_OUT (1)
#define CHX01_INT_DIR_IN  (0)

typedef void (chx01_dev_irq_handler)(void);

class CHx01_dev : public ch_dev_t {
public:
  /*!
   * @brief Class constructor for an undefined chirp object.
   */
  CHx01_dev(void);
  /*!
   * @brief Class constructor for a chirp device object.
   * @param i2c_ref Reference of the Wire to be used
   * @param i2c_addr i2c address of the device
   * @param int1_id ID of the interrupt 1 pin
   * @param int_dir_id ID of the interrupt direction pin
   * @param prog_id ID of the interrupt prog pin
   */
  CHx01_dev(TwoWire &i2c_ref, uint8_t i2c_addr, int int1_id, int int_dir_id, int prog_id);
  /*!
   * @brief Configure the sensor
   * @param group Sensor group the sensor belongs (CHx01 object pointer)
   * @param id Id of the sensor in the group
   * @return 0 if successful
   */
   int begin(ch_group_t* group, int id);
  /*!
   * @brief Set int1 direction
   * @param value CHX01_INT_DIR_OUT for output, CHX01_INT_DIR_IN for input
   */
  void set_int1_dir(bool value);
  /*!
   * @brief Set int1 pin value
   * @param value Interrupt level to be set
   */
  void set_int1(bool value);
  /*!
   * @brief Set prog pin value
   * @param value PROG level to be set
   */
  void set_prog(bool value);
  /*!
   * @brief Enable and attach handler
   * @param handler Interrupt routine to be called
   */
  void enableInterrupt(chx01_dev_irq_handler handler);
  /*!
   * @brief Disable and detach handler
   */
  void disableInterrupt(void);
  /*!
   * @brief Get the i2c
   * @return Pointer to the TwoWire instance
   */
  TwoWire* get_i2c(void);
  /*!
   * @brief Get the i2c address
   * @return I2C address of the sensor
   */
  uint8_t get_i2c_addr(void);
  /*!
   * @brief Get the sensor part number.
   * @return Sensor part number as an integer
   */
  uint16_t part_number(void);

  /*!
   * @brief Get sensor's operating frequency.
   * @return Acoustic operating frequency, in Hz
   */
  uint32_t frequency(void);
  /*!
   * @brief Get sensor's bandwidth.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t bandwidth(void);
  /*!
   * @brief Get the real-time clock calibration value.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t rtc_cal(void);
  /*!
   * @brief Get the real-time clock calibration pulse length.
   * @return RTC pulse length, in ms
   */
  uint16_t rtc_cal_pulse_length(void);
  /*!
   * @brief Get the sensor CPU frequency, in Hz.
   * @return sensor CPU frequency, in Hz
   */
  float cpu_freq(void);
  /*!
   * @brief Get the firmware version description string.
   * @return Pointer to character string describing sensor firmware version
   */
  const char *fw_version(void);
  /*!
   * @brief Check if a measure is available.
   * @return True if a measure is available, false otherwise
   */
  bool data_ready(void);
  /*!
   * @brief Set data_ready for the sensor.
   */
   void set_data_ready(void);
  /*!
   * @brief Get the sensor's max number of samples.
   * @return Sensor's max number of sample
   * This value does not depend on sensor current configuration
   */
  uint16_t get_max_samples(void);
  /*!
   * @brief Get the sensor's max range.
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  virtual uint16_t get_max_range(void);
  /*!
   * @brief Get the sensor's range configured for measures.
   * @return Sensor's measure range in mm
   * This value depends on sensor configured range
   */
  uint16_t get_measure_range(void);
    /*!
   * @brief Configure embedded algo.
   * @return 0 if successful, 1 if error
   */
  virtual int algo_config(void);
  /*!
   * @brief Starts sensor in free running mode, with maximum range and default ODR.
   * @return 0 in case of success
   */
  int free_run();
  /*!
   * @brief Starts sensor in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm, uint16_t interval_ms);
  /*!
   * @brief Starts sensor in trigger mode, with provided max range.
   * @param range_mm Max detection range to be configured
   * @param mode Select sensor mode: CH_MODE_TRIGGERED_TX_RX or CH_MODE_TRIGGERED_RX_ONLY
   * @return 0 in case of success
   */
  int start_trigger(uint16_t range_mm, ch_mode_t mode);
  /*!
   * @brief Trig sensor measurement.
   */
  void trig(void);
  /*!
   * @brief Displays sensor information.
   */
  void print_informations(void);
  /*!
   * @brief Displays sensor configuration.
   */
  void print_configuration(void);
  /*!
   * @brief Get the measured range from a sensor.
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
   float get_range(void);
  /*!
   * @brief Reset sensor i2c.
   */
   void i2c_reset(void);
protected:
  TwoWire* i2c;
  uint8_t i2c_address;
  int int1_pin_id;
  int int_dir_pin_id;
  int prog_pin_id;
  const uint32_t default_odr_ms = 100; // 1 measure each 100 ms
  ch_fw_init_func_t fw_init_func;
  bool int1_attached;
  bool is_measure_ready;
  void clear_data_ready(void);

};

#endif // CHx01_DEV_H
