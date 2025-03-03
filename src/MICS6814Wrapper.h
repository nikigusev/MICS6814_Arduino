#ifndef MICS6814WRAPPER_H
#define MICS6814WRAPPER_H

#include <Arduino.h>
#include "PimoroniI2C.h"
#include "IOExpander.h"
#include "MICS6814.h"

// Wrapper class that hides the underlying I2C and IOExpander details
class MICS6814Wrapper {
public:
  // Constructor with optional debug flag, I2C address, and brightness.
  // Default parameters: debug = false, address = 0x19, brightness = 1.0
  MICS6814Wrapper(bool debug = false, uint8_t address = 0x19, float brightness = 1.0f)
    : i2c(), 
      ioExpander(&i2c, address, IOExpander::PIN_UNUSED, false, 1000, debug),
      sensor(ioExpander, brightness)
  {
  }

  bool init(bool skip_chip_id_check = false) {
    i2c.init();
    return sensor.init(skip_chip_id_check);
  }
  
  void set_led(uint8_t r, uint8_t g, uint8_t b) {
    sensor.set_led(r, g, b);
  }
  
  void set_heater(bool on) {
    sensor.set_heater(on);
  }
  
  void disable_heater() {
    sensor.disable_heater();
  }
  
  float read_reducing(uint32_t timeout = 1000) {
    return sensor.read_reducing(timeout);
  }
  
  float read_nh3(uint32_t timeout = 1000) {
    return sensor.read_nh3(timeout);
  }
  
  float read_oxidising(uint32_t timeout = 1000) {
    return sensor.read_oxidising(timeout);
  }
  
  MICS6814::Reading read_all(uint32_t timeout = 1000) {
    return sensor.read_all(timeout);
  }

private:
  PimoroniI2C i2c;
  IOExpander ioExpander;
  MICS6814 sensor;
};

#endif
