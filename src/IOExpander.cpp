#include "IOExpander.h"

// We replicate the entire pimoroni ioexpander.cpp logic in detail:

// These regs come from the original code:
namespace
{
  // Basic chip identity
  const uint8_t REG_CHIP_ID_L = 0xfa;
  const uint8_t REG_CHIP_ID_H = 0xfb;
  const uint8_t REG_VERSION = 0xfc;
  const uint8_t REG_ADDR = 0xfd;
  const uint8_t REG_CTRL = 0xfe;
  const uint8_t REG_INT = 0xf9; // bit0=TRIGD, bit1=OUT_EN, bit2=PIN_SWAP

  // Bits in the CTRL register
  const uint8_t CTRL_SLEEP = 1 << 0;
  const uint8_t CTRL_RESET = 1 << 1;
  const uint8_t CTRL_FREAD = 1 << 2;
  const uint8_t CTRL_FWRITE = 1 << 3;
  const uint8_t CTRL_ADDRWR = 1 << 4;

  // Bits in the INT register
  const uint8_t INT_BIT_TRIGGERED = 0;
  const uint8_t INT_BIT_OUT_EN = 1;
  const uint8_t INT_BIT_PIN_SWAP = 2;

  // user flash reg for checking reset
  const uint8_t REG_USER_FLASH = 0xD0;

  // PWM registers
  const uint8_t REG_PWMCON0 = 0x98; // bit7=PWMRUN, bit6=LOAD, bit4=CLRPWM
  const uint8_t REG_PWMCON1 = 0x9f; // bits for clock divider
  const uint8_t REG_PNP = 0x96;     // bit-changes for inversion
  const uint8_t BIT_LOAD = 6;
  const uint8_t BIT_CLRPWM = 4;
  const uint8_t BIT_PWMRUN = 7;

  const uint8_t REG_PWMPH = 0x91;
  const uint8_t REG_PWMPL = 0x99;

  // ADC registers
  const uint8_t REG_ADCCON0 = 0xa8;
  const uint8_t REG_ADCCON1 = 0xa1;
  const uint8_t REG_ADCRH = 0x83;
  const uint8_t REG_ADCRL = 0x82;
  const uint8_t REG_AINDIDS = 0xb6; // for input channel disabling

  // Additional memory for rotary, etc.
  const uint8_t REG_ENC_EN = 0x04;
  const uint8_t REG_ENC_1_CFG = 0x05;
  const uint8_t REG_ENC_1_COUNT = 0x06;
  const uint8_t REG_ENC_2_CFG = 0x07;
  const uint8_t REG_ENC_2_COUNT = 0x08;
  const uint8_t REG_ENC_3_CFG = 0x09;
  const uint8_t REG_ENC_3_COUNT = 0x0A;
  const uint8_t REG_ENC_4_CFG = 0x0B;
  const uint8_t REG_ENC_4_COUNT = 0x0C;

  const uint8_t ENC_CFG[4] = {REG_ENC_1_CFG, REG_ENC_2_CFG, REG_ENC_3_CFG, REG_ENC_4_CFG};
  const uint8_t ENC_COUNT[4] = {REG_ENC_1_COUNT, REG_ENC_2_COUNT, REG_ENC_3_COUNT, REG_ENC_4_COUNT};

  // For bit addressing
  const uint8_t REG_INT_MASK_P0 = 0x00;
  const uint8_t REG_INT_MASK_P1 = 0x01;
  const uint8_t REG_INT_MASK_P3 = 0x03;
}

// The static arrays from pimoroni's code for pin definitions:
static const uint8_t PIN_PxM1[4] = {0x71, 0x73, (uint8_t)-1, 0x6c}; // P0M1, P1M1, invalid, P3M1
static const uint8_t PIN_PxM2[4] = {0x72, 0x74, (uint8_t)-1, 0x6d};
static const uint8_t PIN_Px[4] = {0x40, 0x50, (uint8_t)-1, 0x70};
static const uint8_t PIN_PxS[4] = {0xc2, 0xc4, (uint8_t)-1, 0xc0};
static const uint8_t PIN_MASK_P[4] = {REG_INT_MASK_P0, REG_INT_MASK_P1, (uint8_t)-1, REG_INT_MASK_P3};

// For PWM channels
static const uint8_t PIN_PWML[6] = {0x9a, 0x9b, 0x9c, 0x9d, 0x8c, 0x8d};
static const uint8_t PIN_PWMH[6] = {0x92, 0x93, 0x94, 0x95, 0x84, 0x85};

// For toggling IO vs PWM
// Example: 0x9e => PIOCON0, 0xc9 => PIOCON1
static const uint8_t REG_IO_PWM_0 = 0x9e;
static const uint8_t REG_IO_PWM_1 = 0xc9;

////////////////////////////////////////////////////////
// Implementation of Pin struct
////////////////////////////////////////////////////////
bool IOExpander::Pin::mode_supported(uint8_t desired_mode)
{
  // Extract the base IO type from desired_mode
  // bit2 => PWM, bit3 => ADC
  bool want_adc = (desired_mode & IOExpander::PIN_MODE_ADC) != 0;
  bool want_pwm = (desired_mode & IOExpander::PIN_MODE_PWM) != 0;

  if (want_pwm && !(type & TYPE_PWM))
    return false;
  if (want_adc && !(type & TYPE_ADC))
    return false;
  // If the pin is TYPE_ADC_OR_PWM, it can do either
  return true;
}

void IOExpander::Pin::set_mode(uint8_t new_mode)
{
  mode = new_mode;
}

IOExpander::Pin::IOType IOExpander::Pin::get_type()
{
  return type;
}

uint8_t IOExpander::Pin::get_mode()
{
  return mode;
}

// Constructors
IOExpander::Pin IOExpander::Pin::io(uint8_t port, uint8_t pin)
{
  Pin p;
  p.type = TYPE_IO;
  p.mode = 0;
  p.port = port;
  p.pin = pin;
  p.adc_channel = 0;
  p.pwm_channel = 0;
  p.reg_m1 = PIN_PxM1[port];
  p.reg_m2 = PIN_PxM2[port];
  p.reg_p = PIN_Px[port];
  p.reg_ps = PIN_PxS[port];
  p.reg_int_mask_p = PIN_MASK_P[port];
  p.reg_io_pwm = 0;
  p.reg_pwml = 0;
  p.reg_pwmh = 0;
  return p;
}

IOExpander::Pin IOExpander::Pin::pwm(uint8_t port, uint8_t pin, uint8_t channel, uint8_t reg_iopwm)
{
  Pin p;
  p.type = TYPE_PWM;
  p.mode = 0;
  p.port = port;
  p.pin = pin;
  p.adc_channel = 0;
  p.pwm_channel = channel;
  p.reg_m1 = PIN_PxM1[port];
  p.reg_m2 = PIN_PxM2[port];
  p.reg_p = PIN_Px[port];
  p.reg_ps = PIN_PxS[port];
  p.reg_int_mask_p = PIN_MASK_P[port];
  p.reg_io_pwm = reg_iopwm;
  p.reg_pwml = PIN_PWML[channel];
  p.reg_pwmh = PIN_PWMH[channel];
  return p;
}

IOExpander::Pin IOExpander::Pin::adc(uint8_t port, uint8_t pin, uint8_t channel)
{
  Pin p;
  p.type = TYPE_ADC;
  p.mode = 0;
  p.port = port;
  p.pin = pin;
  p.adc_channel = channel;
  p.pwm_channel = 0;
  p.reg_m1 = PIN_PxM1[port];
  p.reg_m2 = PIN_PxM2[port];
  p.reg_p = PIN_Px[port];
  p.reg_ps = PIN_PxS[port];
  p.reg_int_mask_p = PIN_MASK_P[port];
  p.reg_io_pwm = 0;
  p.reg_pwml = 0;
  p.reg_pwmh = 0;
  return p;
}

IOExpander::Pin IOExpander::Pin::adc_or_pwm(uint8_t port, uint8_t pin, uint8_t adc_channel, uint8_t pwm_channel, uint8_t reg_iopwm)
{
  Pin p;
  p.type = TYPE_ADC_OR_PWM;
  p.mode = 0;
  p.port = port;
  p.pin = pin;
  p.adc_channel = adc_channel;
  p.pwm_channel = pwm_channel;
  p.reg_m1 = PIN_PxM1[port];
  p.reg_m2 = PIN_PxM2[port];
  p.reg_p = PIN_Px[port];
  p.reg_ps = PIN_PxS[port];
  p.reg_int_mask_p = PIN_MASK_P[port];
  p.reg_io_pwm = reg_iopwm;
  p.reg_pwml = PIN_PWML[pwm_channel];
  p.reg_pwmh = PIN_PWMH[pwm_channel];
  return p;
}

////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////

IOExpander::IOExpander(PimoroniI2C *i2c,
                       uint8_t address,
                       uint8_t interruptPin,
                       bool pinSwapOut,
                       uint32_t timeout,
                       bool debug)
    : i2c(i2c), address(address), interruptPin(interruptPin),
      pinSwapOut(pinSwapOut), timeout(timeout), debug(debug), vref(3.3f)
{
  for (int i = 0; i < 4; i++)
  {
    encoder_offset[i] = 0;
    encoder_last[i] = 0;
  }

  // The original code enumerates 14 pins in a specific order. We'll replicate that:
  // This is the same array as pimoroni's "pins{ ... }" from the code:
  pins[0] = Pin::pwm(1, 5, 5, REG_IO_PWM_1); // (port=1,pin=5), pwmCh=5 -> (PWML5/PWMH5)
  pins[1] = Pin::pwm(1, 0, 2, REG_IO_PWM_0);
  pins[2] = Pin::pwm(1, 2, 0, REG_IO_PWM_0);
  pins[3] = Pin::pwm(1, 4, 1, REG_IO_PWM_1);
  pins[4] = Pin::pwm(0, 0, 3, REG_IO_PWM_0);
  pins[5] = Pin::pwm(0, 1, 4, REG_IO_PWM_0);
  // 7 can be ADC or PWM:
  pins[6] = Pin::adc_or_pwm(1, 1, 7, 1, REG_IO_PWM_0);
  pins[7] = Pin::adc_or_pwm(0, 3, 6, 5, REG_IO_PWM_0);
  pins[8] = Pin::adc_or_pwm(0, 4, 5, 3, REG_IO_PWM_1);
  pins[9] = Pin::adc(3, 0, 1);
  pins[10] = Pin::adc(0, 6, 3);
  pins[11] = Pin::adc_or_pwm(0, 5, 4, 2, REG_IO_PWM_1);
  pins[12] = Pin::adc(0, 7, 2);
  pins[13] = Pin::adc(1, 7, 0);
}

////////////////////////////////////////////////////////
// init / reset
////////////////////////////////////////////////////////
bool IOExpander::init(bool skipChipIdCheck, bool perform_reset)
{
  if (!i2c)
  {
    if (debug)
    {
      Serial.println("[IOExpander] i2c pointer is null!");
    }
    return false;
  }
  i2c->init();

  if (!skipChipIdCheck)
  {
    uint16_t id = get_chip_id();
    if (id != CHIP_ID)
    {
      if (debug)
      {
        Serial.print("[IOExpander] Wrong chip ID: 0x");
        Serial.print(id, HEX);
        Serial.print(" (expected 0x");
        Serial.print(CHIP_ID, HEX);
        Serial.println(")");
      }
      return false;
    }
  }

  if (perform_reset)
  {
    if (!reset())
    {
      return false;
    }
  }

  // If we have an Arduino pin for the interrupt, set it as input w/pullup
  if (interruptPin != PIN_UNUSED)
  {
    pinMode(interruptPin, INPUT_PULLUP);
  }

  // enable interrupt out with optional pin swap
  enable_interrupt_out(pinSwapOut);

  return true;
}

bool IOExpander::reset()
{
  // Set the RESET bit in CTRL
  uint8_t c = i2c->reg_read_uint8(address, REG_CTRL);
  c |= CTRL_RESET;
  i2c->reg_write_uint8(address, REG_CTRL, c);

  // Wait for check_reset() to yield 0x78
  uint32_t start_time = board_millis();
  while (check_reset() != 0x78)
  {
    if (board_millis() - start_time >= RESET_TIMEOUT_MS)
    {
      if (debug)
      {
        Serial.println("[IOExpander] Timed out waiting for reset!");
      }
      return false;
    }
    delay(1);
  }
  return true;
}

uint8_t IOExpander::check_reset()
{
  // read the user flash register 0xD0
  uint8_t reg = REG_USER_FLASH;
  if (i2c->write_blocking(address, &reg, 1, false) < 0)
  {
    return 0x00;
  }
  uint8_t value = 0;
  if (i2c->read_blocking(address, &value, 1, false) < 0)
  {
    return 0x00;
  }
  return value;
}

uint16_t IOExpander::get_chip_id()
{
  uint8_t hi = i2c->reg_read_uint8(address, REG_CHIP_ID_H);
  uint8_t lo = i2c->reg_read_uint8(address, REG_CHIP_ID_L);
  return (uint16_t)((hi << 8) | lo);
}

void IOExpander::set_address(uint8_t newAddr)
{
  // The original sets bit4 in CTRL to unlock the address
  uint8_t c = i2c->reg_read_uint8(address, REG_CTRL);
  c |= CTRL_ADDRWR;
  i2c->reg_write_uint8(address, REG_CTRL, c);

  // write new address
  i2c->reg_write_uint8(address, REG_ADDR, newAddr);
  this->address = newAddr;

  delay(250);

  // clear the unlock bit
  c = i2c->reg_read_uint8(address, REG_CTRL);
  c &= ~CTRL_ADDRWR;
  i2c->reg_write_uint8(address, REG_CTRL, c);
}

////////////////////////////////////////////////////////
// ADC reference
////////////////////////////////////////////////////////
float IOExpander::get_adc_vref()
{
  return vref;
}

void IOExpander::set_adc_vref(float newVref)
{
  vref = newVref;
}

////////////////////////////////////////////////////////
// input / output
////////////////////////////////////////////////////////
int16_t IOExpander::input(uint8_t pin, uint32_t adc_timeout)
{
  if (pin < 1 || pin > NUM_PINS)
  {
    if (debug)
    {
      Serial.println("[IOExpander] input(): pin out of range!");
    }
    return -1;
  }
  Pin &p = pins[pin - 1];
  uint8_t current_mode = p.get_mode();

  // If in ADC mode, do an ADC read
  if (current_mode & PIN_MODE_ADC)
  {
    // (1) Read ADCCON0
    uint8_t adccon0 = i2c->reg_read_uint8(address, REG_ADCCON0);

    // (2) Clear the ADCF (conversion finished) flag, bit7
    adccon0 &= ~(1 << 7);
    i2c->reg_write_uint8(address, REG_ADCCON0, adccon0);

    // (3) Enable the ADC hardware by setting bit0 of ADCCON1
    set_bit(REG_ADCCON1, 0);

    // (4) Clear the lower nibble of ADCCON0 (channel select bits)
    adccon0 &= 0xF0;

    // (5) Set the channel bits to p.adc_channel
    adccon0 |= (p.adc_channel & 0x0F);
    i2c->reg_write_uint8(address, REG_ADCCON0, adccon0);

    // (6) Update AINDIDS to reflect the chosen channel
    i2c->reg_write_uint8(address, REG_AINDIDS, 0); // typically set to 0 first
    uint8_t aind = i2c->reg_read_uint8(address, REG_AINDIDS);
    aind |= (1 << p.adc_channel);
    i2c->reg_write_uint8(address, REG_AINDIDS, aind);

    // (7) Start conversion by setting ADCS (bit6) in ADCCON0
    adccon0 |= (1 << 6);
    i2c->reg_write_uint8(address, REG_ADCCON0, adccon0);

    // (8) Wait for ADCF (bit7) to become 1
    uint32_t start = board_millis();
    while (true)
    {
      uint8_t regv = i2c->reg_read_uint8(address, REG_ADCCON0);
      if (regv & (1 << 7))
      {
        // conversion done
        break;
      }
      if (board_millis() - start > adc_timeout)
      {
        if (debug)
        {
          Serial.println("[IOExpander] ADC conversion timeout!");
        }
        return -1;
      }
      delay(1);
    }

    // (9) Read ADCRH & ADCRL => 12 bits
    uint8_t hi = i2c->reg_read_uint8(address, REG_ADCRH);
    uint8_t lo = i2c->reg_read_uint8(address, REG_ADCRL);
    uint16_t raw = ((uint16_t)(hi << 4)) | (lo & 0x0F);

    return (int16_t)raw;
  }
  else
  {
    // Digital read path
    uint8_t bitVal = get_bit(p.reg_p, p.pin) ? 1 : 0;
    return bitVal;
  }
}

float IOExpander::input_as_voltage(uint8_t pin, uint32_t adc_timeout)
{
  int16_t raw = input(pin, adc_timeout);
  if (raw < 0)
    return -1.0f;
  // 12-bit => 0..4095
  float val = (float)raw / 4095.0f;
  return val * vref;
}

void IOExpander::output(uint8_t pin, uint16_t value, bool load, bool wait_for_load)
{
  if (pin < 1 || pin > NUM_PINS)
  {
    if (debug)
    {
      Serial.println("[IOExpander] output(): pin out of range!");
    }
    return;
  }
  Pin &p = pins[pin - 1];
  uint8_t current_mode = p.get_mode();

  // If pin is in PWM mode
  if (current_mode & PIN_MODE_PWM)
  {
    // set PWML, PWMH
    i2c->reg_write_uint8(address, p.reg_pwml, (uint8_t)(value & 0xFF));
    i2c->reg_write_uint8(address, p.reg_pwmh, (uint8_t)((value >> 8) & 0xFF));
    if (load)
    {
      pwm_load(wait_for_load);
    }
  }
  else
  {
    // digital
    if (value == 0)
    {
      // clr_bit(p.reg_p, p.pin)
      clr_bit(p.reg_p, p.pin);
    }
    else
    {
      // set_bit(p.reg_p, p.pin)
      set_bit(p.reg_p, p.pin);
    }
  }
}

////////////////////////////////////////////////////////
// set_mode
////////////////////////////////////////////////////////
uint8_t IOExpander::get_mode(uint8_t pin)
{
  if (pin < 1 || pin > NUM_PINS)
  {
    if (debug)
    {
      Serial.println("[IOExpander] get_mode(): pin out of range");
    }
    return 0xFF;
  }
  return pins[pin - 1].get_mode();
}

void IOExpander::set_mode(uint8_t pin, uint8_t mode, bool schmitt_trigger, bool invert)
{
  if (pin < 1 || pin > NUM_PINS)
  {
    if (debug)
    {
      Serial.println("[IOExpander] set_mode(): pin out of range");
    }
    return;
  }
  Pin &p = pins[pin - 1];

  if (!p.mode_supported(mode))
  {
    if (debug)
    {
      Serial.print("[IOExpander] Pin ");
      Serial.print(pin);
      Serial.println(" does not support requested mode!");
    }
    return;
  }

  p.set_mode(mode);

  // 1) If (mode & PIN_MODE_PWM), set the bit for that PWM channel in p.reg_io_pwm
  if (mode & PIN_MODE_PWM)
  {
    set_bit(p.reg_io_pwm, p.pwm_channel);
    // if we want invert, set PNP bit for that channel
    change_bit(REG_PNP, p.pwm_channel, invert);
    // also set bit7 of PWMCON0 => PWMRUN
    set_bit(REG_PWMCON0, BIT_PWMRUN);
  }
  else
  {
    // If previously was PWM, clear that bit
    if (p.get_type() & Pin::TYPE_PWM)
    {
      clr_bit(p.reg_io_pwm, p.pwm_channel);
    }
  }

  // 2) We must set the correct "P0M1/P0M2" or "P1M1/P1M2" bits to define
  //    Quasi-bidirectional, push-pull, input, or open-drain
  uint8_t pm1 = i2c->reg_read_uint8(address, p.reg_m1);
  uint8_t pm2 = i2c->reg_read_uint8(address, p.reg_m2);

  // Clear bits for that pin
  pm1 &= ~(1 << p.pin);
  pm2 &= ~(1 << p.pin);

  // The lowest 2 bits of mode => 0=QB,1=PP,2=IN,3=OD
  uint8_t gpio_mode = (mode & 0x03);

  // pm1 bit => hi part of that 2-bit field
  // pm2 bit => lo part
  // example: if gpio_mode=2 (binary 10), pm1=1, pm2=0
  // so pm1 bit = (gpio_mode>>1), pm2 bit = (gpio_mode&1)
  pm1 |= ((gpio_mode >> 1) & 0x01) << p.pin;
  pm2 |= (gpio_mode & 0x01) << p.pin;

  i2c->reg_write_uint8(address, p.reg_m1, pm1);
  i2c->reg_write_uint8(address, p.reg_m2, pm2);

  // Schmitt trigger
  if ((gpio_mode == 2) || (gpio_mode == 0x02))
  {
    // input => set bit in p.reg_ps if schmitt_trigger
    change_bit(p.reg_ps, p.pin, schmitt_trigger);
  }

  // If the 5th bit of mode is set => default HIGH
  bool default_high = (mode & 0x10) != 0;
  if (default_high)
  {
    set_bit(p.reg_p, p.pin);
  }
  else
  {
    clr_bit(p.reg_p, p.pin);
  }
}

////////////////////////////////////////////////////////
// PWM methods
////////////////////////////////////////////////////////
bool IOExpander::set_pwm_control(uint8_t divider)
{
  // replicate the big switch
  uint8_t pwmdiv2 = 0;
  switch (divider)
  {
  case 1:
    pwmdiv2 = 0b000;
    break;
  case 2:
    pwmdiv2 = 0b001;
    break;
  case 4:
    pwmdiv2 = 0b010;
    break;
  case 8:
    pwmdiv2 = 0b011;
    break;
  case 16:
    pwmdiv2 = 0b100;
    break;
  case 32:
    pwmdiv2 = 0b101;
    break;
  case 64:
    pwmdiv2 = 0b110;
    break;
  case 128:
    pwmdiv2 = 0b111;
    break;
  default:
    if (debug)
    {
      Serial.print("[IOExpander] invalid PWM divider: ");
      Serial.println(divider);
    }
    return false;
  }
  // write to PWMCON1
  i2c->reg_write_uint8(address, REG_PWMCON1, pwmdiv2);
  return true;
}

void IOExpander::set_pwm_period(uint16_t value, bool load, bool wait_for_load)
{
  i2c->reg_write_uint8(address, REG_PWMPL, (uint8_t)(value & 0xFF));
  i2c->reg_write_uint8(address, REG_PWMPH, (uint8_t)((value >> 8) & 0xFF));
  if (load)
  {
    pwm_load(wait_for_load);
  }
}

uint16_t IOExpander::set_pwm_frequency(float frequency, bool load, bool wait_for_load)
{
  // replicate logic
  uint32_t period = (uint32_t)(CLOCK_FREQ / frequency);
  if ((period / 128) > MAX_PERIOD)
  {
    period = MAX_PERIOD * 128;
  }
  if (period < 2)
  {
    period = 2;
  }
  uint8_t div = 1;
  while ((period > MAX_PERIOD) && (div < MAX_DIVIDER))
  {
    period >>= 1;
    div <<= 1;
  }
  period = min(period, (uint32_t)MAX_PERIOD);

  set_pwm_control(div);
  set_pwm_period((uint16_t)(period - 1), load, wait_for_load);
  return (uint16_t)period;
}

void IOExpander::pwm_load(bool wait_for_load)
{
  uint8_t regv = i2c->reg_read_uint8(address, REG_PWMCON0);
  // set LOAD bit (bit6)
  regv |= (1 << BIT_LOAD);
  i2c->reg_write_uint8(address, REG_PWMCON0, regv);

  if (wait_for_load)
  {
    uint32_t start = board_millis();
    while (pwm_loading())
    {
      if (board_millis() - start > timeout)
      {
        if (debug)
        {
          Serial.println("[IOExpander] pwm_load() timed out!");
        }
        return;
      }
      delay(1);
    }
  }
}

bool IOExpander::pwm_loading()
{
  uint8_t regv = i2c->reg_read_uint8(address, REG_PWMCON0);
  return (regv & (1 << BIT_LOAD)) != 0;
}

void IOExpander::pwm_clear(bool wait_for_clear)
{
  uint8_t regv = i2c->reg_read_uint8(address, REG_PWMCON0);
  // set CLRPWM bit (bit4)
  regv |= (1 << BIT_CLRPWM);
  i2c->reg_write_uint8(address, REG_PWMCON0, regv);

  if (wait_for_clear)
  {
    uint32_t start = board_millis();
    while (pwm_clearing())
    {
      if (board_millis() - start > timeout)
      {
        if (debug)
        {
          Serial.println("[IOExpander] pwm_clear() timed out!");
        }
        return;
      }
      delay(1);
    }
  }
}

bool IOExpander::pwm_clearing()
{
  uint8_t regv = i2c->reg_read_uint8(address, REG_PWMCON0);
  return (regv & (1 << BIT_CLRPWM)) != 0;
}

////////////////////////////////////////////////////////
// Interrupt
////////////////////////////////////////////////////////
void IOExpander::enable_interrupt_out(bool pin_swap)
{
  // set OUT_EN bit, set/clear PIN_SWAP bit
  uint8_t v = i2c->reg_read_uint8(address, REG_INT);
  v |= (1 << INT_BIT_OUT_EN);
  if (pin_swap)
    v |= (1 << INT_BIT_PIN_SWAP);
  else
    v &= ~(1 << INT_BIT_PIN_SWAP);
  i2c->reg_write_uint8(address, REG_INT, v);
}

void IOExpander::disable_interrupt_out()
{
  uint8_t v = i2c->reg_read_uint8(address, REG_INT);
  v &= ~(1 << INT_BIT_OUT_EN);
  i2c->reg_write_uint8(address, REG_INT, v);
}

bool IOExpander::get_interrupt_flag()
{
  // if we have a hardware pin, read that (the original code inverts it)
  if (interruptPin != PIN_UNUSED)
  {
    return (digitalRead(interruptPin) == LOW);
  }
  else
  {
    // fallback to reading the TRIGD bit
    uint8_t v = i2c->reg_read_uint8(address, REG_INT);
    return (v & (1 << INT_BIT_TRIGGERED)) != 0;
  }
}

void IOExpander::clear_interrupt_flag()
{
  uint8_t v = i2c->reg_read_uint8(address, REG_INT);
  v &= ~(1 << INT_BIT_TRIGGERED);
  i2c->reg_write_uint8(address, REG_INT, v);
}

bool IOExpander::set_pin_interrupt(uint8_t pin, bool enabled)
{
  // replicate the original code:
  if (pin < 1 || pin > NUM_PINS)
  {
    return false;
  }
  Pin &p = pins[pin - 1];
  // flip that bit in p.reg_int_mask_p
  change_bit(p.reg_int_mask_p, p.pin, enabled);
  return true;
}

////////////////////////////////////////////////////////
// Rotary encoders
////////////////////////////////////////////////////////
void IOExpander::setup_rotary_encoder(uint8_t channel, uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, bool count_microsteps)
{
  channel -= 1; // convert to 0-based
  // set pins to input pull-up, etc.
  set_mode(pin_a, PIN_MODE_IN | 0x10); // set default=HIGH for input with pull-up
  set_mode(pin_b, PIN_MODE_IN | 0x10);

  if (pin_c != 0)
  {
    set_mode(pin_c, PIN_MODE_OD);
    output(pin_c, 0);
  }

  // write to ENC_CFG
  i2c->reg_write_uint8(address, ENC_CFG[channel], (pin_a & 0x0f) | ((pin_b & 0x0f) << 4));
  // set or clear microstep
  change_bit(REG_ENC_EN, (channel * 2) + 1, count_microsteps);

  // enable encoder
  set_bit(REG_ENC_EN, channel * 2);

  // reset the internal count
  i2c->reg_write_uint8(address, ENC_COUNT[channel], 0x00);
}

int16_t IOExpander::read_rotary_encoder(uint8_t channel)
{
  channel -= 1;
  int16_t last = encoder_last[channel];
  // read the count reg
  uint8_t val = i2c->reg_read_uint8(address, ENC_COUNT[channel]);
  int16_t value = (val & 0x80) ? (val - 256) : val; // sign-extend

  // handle wrap
  if (last > 64 && value < -64)
    encoder_offset[channel] += 256;
  if (last < -64 && value > 64)
    encoder_offset[channel] -= 256;

  encoder_last[channel] = value;
  return encoder_offset[channel] + value;
}

void IOExpander::clear_rotary_encoder(uint8_t channel)
{
  channel -= 1;
  encoder_last[channel] = 0;
  i2c->reg_write_uint8(address, ENC_COUNT[channel], 0);
}

////////////////////////////////////////////////////////
// wait_for_flash
////////////////////////////////////////////////////////
void IOExpander::wait_for_flash()
{
  // The original code toggles the hardware interrupt pin to detect
  // a flash write finishing. We'll replicate that logic as best we can:
  uint32_t start = board_millis();
  // while the device signals busy
  while (get_interrupt_flag())
  {
    if (board_millis() - start > timeout)
    {
      if (debug)
      {
        Serial.println("[IOExpander] Timed out waiting for flash (1)!");
      }
      return;
    }
    delay(1);
  }
  start = board_millis();
  while (!get_interrupt_flag())
  {
    if (board_millis() - start > timeout)
    {
      if (debug)
      {
        Serial.println("[IOExpander] Timed out waiting for flash (2)!");
      }
      return;
    }
    delay(1);
  }
}

////////////////////////////////////////////////////////
// bit helpers
////////////////////////////////////////////////////////
uint8_t IOExpander::get_bit(uint8_t reg, uint8_t bit)
{
  // If the register is P0/P1/P2/P3, we must do bit addressing
  // The pimoroni code writes e.g. 0b1000|(bit&0b111). We'll do direct:
  // We'll do a read, then test that bit.
  uint8_t v = i2c->reg_read_uint8(address, reg);
  return (v & (1 << bit));
}

void IOExpander::set_bit(uint8_t reg, uint8_t bit)
{
  // For P0..P3, we can do the 0b1000 | bit approach. Let’s replicate that logic:
  if (reg == 0x40 || reg == 0x50 || reg == 0x60 || reg == 0x70)
  {
    // bit addressing for set
    // we write 0b1000|(bit & 0b111) to the register
    uint8_t cmd = 0b1000 | (bit & 0b111);
    i2c->write_blocking(address, &reg, 1, true); // set reg pointer
    i2c->write_blocking(address, &cmd, 1, false);
  }
  else
  {
    uint8_t val = i2c->reg_read_uint8(address, reg);
    val |= (1 << bit);
    i2c->reg_write_uint8(address, reg, val);
  }
}

void IOExpander::clr_bit(uint8_t reg, uint8_t bit)
{
  if (reg == 0x40 || reg == 0x50 || reg == 0x60 || reg == 0x70)
  {
    // bit addressing for clear
    uint8_t cmd = (bit & 0b111);
    i2c->write_blocking(address, &reg, 1, true);
    i2c->write_blocking(address, &cmd, 1, false);
  }
  else
  {
    uint8_t val = i2c->reg_read_uint8(address, reg);
    val &= ~(1 << bit);
    i2c->reg_write_uint8(address, reg, val);
  }
}

void IOExpander::set_bits(uint8_t reg, uint8_t bits)
{
  // replicate the original logic
  uint8_t val = i2c->reg_read_uint8(address, reg);
  val |= bits;
  i2c->reg_write_uint8(address, reg, val);
}

void IOExpander::clr_bits(uint8_t reg, uint8_t bits)
{
  uint8_t val = i2c->reg_read_uint8(address, reg);
  val &= ~bits;
  i2c->reg_write_uint8(address, reg, val);
}

void IOExpander::change_bit(uint8_t reg, uint8_t bit, bool state)
{
  if (state)
    set_bit(reg, bit);
  else
    clr_bit(reg, bit);
}
