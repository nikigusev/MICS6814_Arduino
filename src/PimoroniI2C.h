#pragma once

#include <Arduino.h>
#include <Wire.h>

/*
 * A thorough Arduino-based I2C helper class,
 * replicating Pimoroni's I2C read/write conventions.
 */

class PimoroniI2C {
public:
  PimoroniI2C(TwoWire &wireInstance = Wire, uint32_t frequency = 100000)
    : wire(wireInstance), freq(frequency) {}

  // Call once in setup to init the bus
  void init() {
    wire.begin();       // SDA/SCL automatically chosen for Arduino
    wire.setClock(freq);
  }

  // Write 'len' bytes from 'src' to device 'addr'. If 'nostop' is true, use repeated start.
  int write_blocking(uint8_t addr, const uint8_t *src, size_t len, bool nostop);

  // Read 'len' bytes into 'dst' from device 'addr'. If 'nostop' is true, use repeated start.
  int read_blocking(uint8_t addr, uint8_t *dst, size_t len, bool nostop);

  // Write a single byte to register
  void reg_write_uint8(uint8_t address, uint8_t reg, uint8_t value);

  // Write a 16-bit value (MSB first) to register
  void reg_write_uint16(uint8_t address, uint8_t reg, uint16_t value);

  // Read a single byte from register
  uint8_t  reg_read_uint8(uint8_t address, uint8_t reg);

  // Read a 16-bit value (2 bytes) from register
  uint16_t reg_read_uint16(uint8_t address, uint8_t reg);

  // Read a 32-bit value (4 bytes) from register
  uint32_t reg_read_uint32(uint8_t address, uint8_t reg);

  // Read a signed 16-bit value
  int16_t  reg_read_int16(uint8_t address, uint8_t reg);

  // Write multiple bytes into register
  int write_bytes(uint8_t address, uint8_t reg, const uint8_t *buf, int len);

  // Read multiple bytes from register into buf
  int read_bytes(uint8_t address, uint8_t reg, uint8_t *buf, int len);

  // Helper to set/clear bits in a register
  uint8_t get_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);
  void    set_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);
  void    clear_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);

private:
  TwoWire &wire;
  uint32_t freq;
};
