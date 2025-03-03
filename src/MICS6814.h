#pragma once

#include <Arduino.h>
#include "IOExpander.h"

/*
 * A thorough port of Pimoroni's breakout_mics6814.hpp
 * for the MICS6814 3-in-1 Gas Sensor, using the
 * Nuvoton MS51 IOExpander (PIM569).
 * 
 * Fully replicates:
 *  - LED brightness & color (via PWM)
 *  - Heater control
 *  - Reading raw ADC for the three gas channels (reducing, NH3, oxidising)
 *  - "read_all()" returning ohms for each sensor
 */

class MICS6814 {
public:
  struct Reading {
    float reducing;   // ohms
    float nh3;        // ohms
    float oxidising;  // ohms
    float ref;        // reference voltage
  };

  // Provide an IOExpander reference + optional LED brightness
  MICS6814(IOExpander &ioe, float brightness = 1.0f);

  bool init(bool skip_chip_id_check = false);

  // Access the underlying IOExpander if needed
  IOExpander &getIOExpander() { return ioe; }

  // Adjust the brightness factor (0.01 .. 1.0)
  void set_brightness(float brightness);

  // Set LED color with 8-bit channels (0..255).
  // This uses the MS51's internal PWM for each color.
  void set_led(uint8_t r, uint8_t g, uint8_t b);

  // Heater control
  void set_heater(bool on);
  void disable_heater(); // sets heater pin to input

  // Direct ADC reads (in volts)
  float get_raw_ref(uint32_t adc_timeout = 1000);
  float get_raw_red(uint32_t adc_timeout = 1000);
  float get_raw_nh3(uint32_t adc_timeout = 1000);
  float get_raw_oxd(uint32_t adc_timeout = 1000);

  // Convert measured voltages into resistance (ohms)
  float read_ref(uint32_t adc_timeout = 1000);
  float read_reducing(uint32_t adc_timeout = 1000);
  float read_nh3(uint32_t adc_timeout = 1000);
  float read_oxidising(uint32_t adc_timeout = 1000);

  // Read all channels at once
  Reading read_all(uint32_t adc_timeout = 1000);

private:
  IOExpander &ioe;
  float brightness;

  static const uint8_t MICS_VREF      = 14;   // maps to pins[13]: ADC channel from Port1, pin7, channel 0
  static const uint8_t MICS_RED       = 13;   // maps to pins[12]: ADC channel from Port0, pin7, channel 2
  static const uint8_t MICS_NH3       = 11;   // maps to pins[10]: ADC channel from Port0, pin6, channel 3
  static const uint8_t MICS_OX        = 12;   // maps to pins[11]: ADC_OR_PWM from Port0, pin5, channel 4, PWM channel 2
  static const uint8_t MICS_HEATER_EN = 1;    // maps to pins[0]: PWM channel from Port1, pin5, channel 5

  static const uint8_t LED_R = 3;  // as originally defined (e.g. Port1, pin4, PWM channel 1)
  static const uint8_t LED_G = 7;  // as originally defined (e.g. Port0, pin3, ADC_OR_PWM)
  static const uint8_t LED_B = 2;  // as originally defined (e.g. Port1, pin2, PWM channel 0)
};
