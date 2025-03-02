#include "MICS6814.h"

MICS6814::MICS6814(IOExpander &ioe, float brightness)
  : ioe(ioe), brightness(brightness) {
}

bool MICS6814::init(bool skip_chip_id_check) {
  if(!ioe.init(skip_chip_id_check)) {
    return false;
  }

  // Configure the four ADC pins
  ioe.set_mode(MICS_VREF, IOExpander::PIN_MODE_ADC);
  ioe.set_mode(MICS_RED,  IOExpander::PIN_MODE_ADC);
  ioe.set_mode(MICS_NH3,  IOExpander::PIN_MODE_ADC);
  ioe.set_mode(MICS_OX,   IOExpander::PIN_MODE_ADC);

  // Heater pin as open-drain, default OFF => high
  ioe.set_mode(MICS_HEATER_EN, IOExpander::PIN_MODE_OD);
  ioe.output(MICS_HEATER_EN, 1);

  // Set up LED pins as PWM
  uint16_t period = (uint16_t)(255.0f / brightness);
  ioe.set_pwm_period(period);
  ioe.set_pwm_control(2); // fast
  ioe.set_mode(LED_R, IOExpander::PIN_MODE_PWM);
  ioe.set_mode(LED_G, IOExpander::PIN_MODE_PWM);
  ioe.set_mode(LED_B, IOExpander::PIN_MODE_PWM);

  return true;
}

void MICS6814::set_brightness(float b) {
  // clamp
  if(b < 0.01f) b = 0.01f;
  if(b > 1.0f)  b = 1.0f;
  brightness = b;
  uint16_t period = (uint16_t)(255.0f / brightness);
  ioe.set_pwm_period(period);
}

void MICS6814::set_led(uint8_t r, uint8_t g, uint8_t b) {
  // The original writes R/G with load=false, B with load=true
  ioe.output(LED_R, r, false);
  ioe.output(LED_G, g, false);
  ioe.output(LED_B, b, true);
}

void MICS6814::set_heater(bool on) {
  // on => output LOW (since pin is OD)
  // off => output HIGH
  ioe.output(MICS_HEATER_EN, on ? 0 : 1);
}

void MICS6814::disable_heater() {
  // set to input
  ioe.output(MICS_HEATER_EN, 1); // ensure off
  ioe.set_mode(MICS_HEATER_EN, IOExpander::PIN_MODE_IN);
}

float MICS6814::get_raw_ref(uint32_t adc_timeout) {
  return ioe.input_as_voltage(MICS_VREF, adc_timeout);
}
float MICS6814::get_raw_red(uint32_t adc_timeout) {
  return ioe.input_as_voltage(MICS_RED, adc_timeout);
}
float MICS6814::get_raw_nh3(uint32_t adc_timeout) {
  return ioe.input_as_voltage(MICS_NH3, adc_timeout);
}
float MICS6814::get_raw_oxd(uint32_t adc_timeout) {
  return ioe.input_as_voltage(MICS_OX, adc_timeout);
}

float MICS6814::read_ref(uint32_t adc_timeout) {
  float ref = get_raw_ref(adc_timeout);
  if(ref < 0) ref = 0;
  return ref;
}

float MICS6814::read_reducing(uint32_t adc_timeout) {
  float vref = ioe.get_adc_vref();
  float val  = get_raw_red(adc_timeout);
  if(val < 0) {
    return 0.0f;
  }
  if(fabsf(vref - val) < 1e-9f) {
    return 0.0f;
  }
  return (val * 56000.0f) / (vref - val);
}

float MICS6814::read_nh3(uint32_t adc_timeout) {
  float vref = ioe.get_adc_vref();
  float val  = get_raw_nh3(adc_timeout);
  if(val < 0) {
    return 0.0f;
  }
  if(fabsf(vref - val) < 1e-9f) {
    return 0.0f;
  }
  return (val * 56000.0f) / (vref - val);
}

float MICS6814::read_oxidising(uint32_t adc_timeout) {
  float vref = ioe.get_adc_vref();
  float val  = get_raw_oxd(adc_timeout);
  if(val < 0) {
    return 0.0f;
  }
  if(fabsf(vref - val) < 1e-9f) {
    return 0.0f;
  }
  return (val * 56000.0f) / (vref - val);
}

MICS6814::Reading MICS6814::read_all(uint32_t adc_timeout) {
  Reading r;
  r.reducing   = read_reducing(adc_timeout);
  r.nh3        = read_nh3(adc_timeout);
  r.oxidising  = read_oxidising(adc_timeout);
  r.ref        = read_ref(adc_timeout);
  return r;
}
