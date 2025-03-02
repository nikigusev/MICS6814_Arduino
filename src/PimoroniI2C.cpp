#include "PimoroniI2C.h"

int PimoroniI2C::write_blocking(uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
  wire.beginTransmission(addr);
  wire.write(src, len);
  uint8_t status = wire.endTransmission(!nostop);
  // If transmission succeeded, return number of bytes, else -1
  return (status == 0) ? (int)len : -1;
}

int PimoroniI2C::read_blocking(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
  // Request 'len' bytes
  int requested = wire.requestFrom((int)addr, (int)len, (int)!nostop);
  if(requested < (int)len) {
    // Not all bytes arrived
    return -1;
  }
  for(size_t i = 0; i < len; i++) {
    dst[i] = wire.read();
  }
  return (int)len;
}

void PimoroniI2C::reg_write_uint8(uint8_t address, uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  write_blocking(address, buffer, 2, false);
}

void PimoroniI2C::reg_write_uint16(uint8_t address, uint8_t reg, uint16_t value) {
  uint8_t buffer[3];
  buffer[0] = reg;
  buffer[1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[2] = (uint8_t)(value & 0xFF);
  write_blocking(address, buffer, 3, false);
}

uint8_t PimoroniI2C::reg_read_uint8(uint8_t address, uint8_t reg) {
  write_blocking(address, &reg, 1, true);
  uint8_t value = 0;
  read_blocking(address, &value, 1, false);
  return value;
}

uint16_t PimoroniI2C::reg_read_uint16(uint8_t address, uint8_t reg) {
  write_blocking(address, &reg, 1, true);
  uint8_t buf[2] = {0, 0};
  read_blocking(address, buf, 2, false);
  // Pimoroni code typically expects low byte first, then high.
  // However, watch out: Some calls might differ. We'll keep the
  // same approach from their code: (lo + (hi << 8)).
  return (uint16_t)((buf[1] << 8) | buf[0]);
}

uint32_t PimoroniI2C::reg_read_uint32(uint8_t address, uint8_t reg) {
  write_blocking(address, &reg, 1, true);
  uint8_t buf[4] = {0, 0, 0, 0};
  read_blocking(address, buf, 4, false);
  // Return in the same order the Pico code reads them (LSB->MSB):
  return (uint32_t)((buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]);
}

int16_t PimoroniI2C::reg_read_int16(uint8_t address, uint8_t reg) {
  uint16_t raw = reg_read_uint16(address, reg);
  return (int16_t)raw;
}

int PimoroniI2C::write_bytes(uint8_t address, uint8_t reg, const uint8_t *buf, int len) {
  uint8_t temp[len+1];
  temp[0] = reg;
  for(int i = 0; i < len; i++) {
    temp[i+1] = buf[i];
  }
  return write_blocking(address, temp, len+1, false);
}

int PimoroniI2C::read_bytes(uint8_t address, uint8_t reg, uint8_t *buf, int len) {
  write_blocking(address, &reg, 1, true);
  return read_blocking(address, buf, len, false);
}

uint8_t PimoroniI2C::get_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask) {
  uint8_t value = reg_read_uint8(address, reg);
  return (value >> shift) & mask;
}

void PimoroniI2C::set_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask) {
  uint8_t value = reg_read_uint8(address, reg);
  value |= (mask << shift);
  reg_write_uint8(address, reg, value);
}

void PimoroniI2C::clear_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask) {
  uint8_t value = reg_read_uint8(address, reg);
  value &= ~(mask << shift);
  reg_write_uint8(address, reg, value);
}
