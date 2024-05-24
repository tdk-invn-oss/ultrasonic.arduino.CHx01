#include "CHx01_dev.h"
#include "invn/soniclib/ch_rangefinder.h"
#include "invn/soniclib/sensor_fw/ch101/ch101_gpr.h"

CHx01_dev::CHx01_dev(TwoWire& i2c_ref, uint8_t i2c_addr, int int1_id, int int_dir_id, int prog_id) {
  i2c = &i2c_ref;
  i2c_address = i2c_addr;
  int1_pin_id = int1_id;
  int_dir_pin_id = int_dir_id;
  prog_pin_id = prog_id;
  fw_init_func = NULL;
  int1_attached = false;
  is_measure_ready = false;
}


CHx01_dev::CHx01_dev(void) {
  i2c = NULL;
  i2c_address = 0;
  int1_pin_id = UNUSED_PIN;
  int_dir_pin_id = UNUSED_PIN;
  prog_pin_id = UNUSED_PIN;
  fw_init_func = NULL;
  int1_attached = false;
  is_measure_ready = false;
  //memset(this, 0, sizeof(ch_dev_t));
}


int CHx01_dev::begin(ch_group_t* group_ptr, int id)
{
  if(fw_init_func == NULL)
  {
    return -1;
  }

  /* Configure INT pins as input */
  set_int1_dir(CHX01_INT_DIR_IN);

  /* Enable pull-ups on the INT pins */
  pinMode(int1_pin_id, INPUT_PULLUP);

  /* Configure INT DIR pin as output */
  if(int_dir_pin_id != UNUSED_PIN)
  {
    pinMode(int_dir_pin_id, OUTPUT);
    digitalWrite(int_dir_pin_id,CHX01_INT_DIR_IN);
  }

  /* Configure Prog pin as output, set to 0 */
  pinMode(prog_pin_id, OUTPUT);
  digitalWrite(prog_pin_id,LOW);
  
  return ch_init(this, group_ptr, id, fw_init_func);
}

uint16_t CHx01_dev::get_max_samples(void) {
  return ch_get_max_samples(this);
};

uint16_t CHx01_dev::get_max_range(void) {
  return ch_samples_to_mm(this, ch_get_max_samples(this));
};

uint16_t CHx01_dev::get_measure_range(void) {
  uint16_t nb_samples = ch_get_num_samples(this);
  return ch_samples_to_mm(this, nb_samples);
};


int CHx01_dev::free_run(void) { return free_run(get_max_range()); }

int CHx01_dev::free_run(uint16_t range_mm) {
  return free_run(range_mm, default_odr_ms);
}

int CHx01_dev::algo_config(void)
{
  ch_rangefinder_algo_config_t algo_config;
  algo_config.static_range = 0;
  algo_config.thresh_ptr = NULL;
  return ch_rangefinder_set_algo_config(this, &algo_config);
}

int CHx01_dev::free_run(uint16_t range_mm, uint16_t interval_ms) {
  int rc;

  rc = ch_set_max_range(this, range_mm);

  if (rc == 0) {
    rc = ch_set_freerun_interval(this, interval_ms);
  }
  if (rc == 0) {
    rc = ch_freerun_time_hop_enable(this);
  }
  /* Apply GPR configuration */
  if (rc == 0) {
    rc = algo_config();
  }
  if (rc == 0) {
    rc = ch_set_mode(this, CH_MODE_FREERUN);
  }
  if (rc == 0) {
    chdrv_int_set_dir_in(this);
    chdrv_int_interrupt_enable(this);
  }

  return rc;
}

int CHx01_dev::start_trigger(uint16_t range_mm, ch_mode_t mode) {
  int rc;

  rc = ch_set_max_range(this, range_mm);

  /* Apply GPR configuration */
  if (rc == 0) {
    rc = algo_config();
  }
  if (rc == 0) {
    rc = ch_set_mode(this, mode);
  }

  return rc;
}

void CHx01_dev::trig(void) {
  ch_trigger(this);
}

bool CHx01_dev::data_ready(void) { return is_measure_ready; }

void CHx01_dev::set_data_ready(void) { is_measure_ready = true; }
void CHx01_dev::clear_data_ready(void) { is_measure_ready = false; }

uint16_t CHx01_dev::part_number(void) {
  return ch_get_part_number(this);
}

uint32_t CHx01_dev::frequency(void) { return ch_get_frequency(this); }

uint16_t CHx01_dev::bandwidth(void) { return ch_get_bandwidth(this); }

uint16_t CHx01_dev::rtc_cal(void) {
  return ch_get_rtc_cal_result(this);
}

uint16_t CHx01_dev::rtc_cal_pulse_length(void) {
  return ch_get_rtc_cal_pulselength(this);
}

float CHx01_dev::cpu_freq(void) {
  return ch_get_cpu_frequency(this) / 1000000.0f;
}
const char *CHx01_dev::fw_version(void) {
  return ch_get_fw_version_string(this);
}

void CHx01_dev::print_informations(void) {
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
  Serial.println(ch_get_max_samples(this));
}

void CHx01_dev::print_configuration(void) {
  Serial.println("Sensor configuration");
  Serial.print("    measure range (mm): ");
  Serial.println(get_measure_range());
}

float CHx01_dev::get_range(void) {
  float range_mm = 0.0;
  uint32_t range_q5;

  if (data_ready()) {
    range_q5 = ch_get_range(this, (ch_get_mode(this) == CH_MODE_TRIGGERED_RX_ONLY)? (CH_RANGE_DIRECT) : (CH_RANGE_ECHO_ONE_WAY));
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

void CHx01_dev::i2c_reset(void)
{
  i2c->begin();
  i2c->setClock(DEFAULT_I2C_CLOCK);
}

void CHx01_dev::set_int1_dir(bool value)
{

  if(value == CHX01_INT_DIR_IN)
  {

    if(int_dir_pin_id != UNUSED_PIN)
    {
      digitalWrite(int_dir_pin_id, CHX01_INT_DIR_IN);
    }
    pinMode(int1_pin_id,INPUT_PULLUP);
  } else if (value == CHX01_INT_DIR_OUT){
    if(int_dir_pin_id != UNUSED_PIN)
    {
      digitalWrite(int_dir_pin_id,CHX01_INT_DIR_OUT);
    }
    pinMode(int1_pin_id,OUTPUT);
  }
}

void CHx01_dev::set_int1(bool value)
{
  digitalWrite(int1_pin_id, value);
}

void CHx01_dev::set_prog(bool value)
{
  digitalWrite(prog_pin_id, value);
}

void CHx01_dev::enableInterrupt(chx01_dev_irq_handler handler)
{
  if (!int1_attached) {
    set_int1_dir(CHX01_INT_DIR_IN);

    attachInterrupt(digitalPinToInterrupt(int1_pin_id),handler,FALLING);
    int1_attached = true;
  }
}

void CHx01_dev::disableInterrupt(void)
{
  if (int1_attached) {
    detachInterrupt(digitalPinToInterrupt(int1_pin_id));
    int1_attached = false;
  }
}

uint8_t CHx01_dev::get_i2c_addr(void)
{
  return i2c_address;
}

TwoWire* CHx01_dev::get_i2c(void)
{
  return i2c;
}

