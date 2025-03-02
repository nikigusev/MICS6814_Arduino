#pragma once

#include <Arduino.h>
#include "PimoroniI2C.h"

/*
 * A port of Pimoroni's IOExpander (Nuvoton MS51) driver, with
 * FULL interrupt pin handling. This code re-implements all
 * features from the original Pico version, including:
 *  - pin modes (GPIO, ADC, PWM, open-drain, etc.)
 *  - reading/writing digital, PWM, ADC
 *  - interrupt management (enable_interrupt_out, set_pin_interrupt)
 *  - reset, custom i2c address, etc.
 */

class IOExpander {
public:
  // The expected chip ID is 0xe26a from Pimoroni's docs
  static const uint16_t CHIP_ID   = 0xe26a;
  static const uint8_t  NUM_PINS  = 14; // P0..P1..P3-based pins

  // Some pin mode bit patterns used in pimoroni code
  // See original code for how these bits break down:
  //   (lowest 2 bits -> Quasi-bidirectional=0,PP=1,Input=2,OD=3)
  //   bit2 -> 1 => PWM
  //   bit3 -> 1 => ADC
  //   bit4 -> default output state
  static const uint8_t PIN_MODE_QUASI  = 0x00; // (lowest bits=0)
  static const uint8_t PIN_MODE_PP     = 0x01; // push-pull
  static const uint8_t PIN_MODE_IN     = 0x02; // input
  static const uint8_t PIN_MODE_OD     = 0x03; // open-drain
  static const uint8_t PIN_MODE_PWM    = 0x04;
  static const uint8_t PIN_MODE_ADC    = 0x08;
  // Example combos:
  //   (PIN_MODE_IN | PIN_MODE_ADC)  =>  0x0A is "ADC input"
  //   (PIN_MODE_OD | PIN_MODE_PWM)  =>  0x07 is "PWM open-drain" etc.

  // For convenience, define some simpler aliases:
  // (the actual code used something like PIN_MODE_ADC = 0x08, etc.)
  // We'll keep these distinct so we can replicate all usage.
  static const uint8_t PIN_UNUSED = 0xFF;

  // The clock freq for the MS51: 48MHz
  static const uint32_t CLOCK_FREQ = 48000000;
  static const uint16_t MAX_PERIOD = 0xFFFF;
  static const uint8_t  MAX_DIVIDER = 128;

  // Timeout for waits
  static const uint32_t RESET_TIMEOUT_MS = 1000;

  // Create an IOExpander object
  //  i2c         = pointer to a PimoroniI2C instance
  //  address     = I2C addr (default 0x18 or 0x19)
  //  interrupt   = Arduino pin number for the IOExpander's INT pin
  //  pinSwapOut  = whether to set "PIN_SWAP" bit in INT reg (original code)
  //  timeout     = general timeout for resets, etc.
  //  debug       = enable debug prints
  IOExpander(PimoroniI2C *i2c,
             uint8_t address       = 0x19,
             uint8_t interruptPin  = PIN_UNUSED,
             bool    pinSwapOut    = false,
             uint32_t timeout      = 1000,
             bool    debug         = false);

  // Initialize the IOExpander:
  //  - optionally skip the chip ID check
  //  - optionally perform a reset
  bool init(bool skipChipIdCheck = false, bool perform_reset = false);

  // Perform a soft reset
  bool reset();

  // The chip ID read (0xfa + 0xfb)
  uint16_t get_chip_id();

  // Change the IOExpander's own I2C address
  void set_address(uint8_t address);

  // Return the current ADC reference used for input_as_voltage
  float get_adc_vref();

  // Set the reference voltage for ADC conversions
  void set_adc_vref(float vref);

  // Digital/ADC read
  // If pin is in ADC mode, returns raw 12-bit ADC value (0..4095).
  // If digital, returns 0 or 1
  int16_t input(uint8_t pin, uint32_t adc_timeout = 1000);

  // Same as above, but returns a float voltage from 0..vref
  float input_as_voltage(uint8_t pin, uint32_t adc_timeout = 1000);

  // Set pin output:
  //   - if pin is PWM, 'value' is the PWM duty
  //   - else, 0=LOW, non-zero=HIGH for digital out
  // 'load' triggers a PWM load if true
  void output(uint8_t pin, uint16_t value, bool load = true, bool wait_for_load = false);

  // Pin mode getters/setters
  uint8_t get_mode(uint8_t pin);
  void    set_mode(uint8_t pin, uint8_t mode, bool schmitt_trigger = false, bool invert = false);

  // PWM
  bool     set_pwm_control(uint8_t divider);
  void     set_pwm_period(uint16_t value, bool load = true, bool wait_for_load = false);
  uint16_t set_pwm_frequency(float frequency, bool load = true, bool wait_for_load = false);

  void pwm_load(bool wait_for_load = false);
  bool pwm_loading();

  void pwm_clear(bool wait_for_clear = false);
  bool pwm_clearing();

  // Interrupt out
  void enable_interrupt_out(bool pin_swap = false);
  void disable_interrupt_out();
  bool get_interrupt_flag();
  void clear_interrupt_flag();
  bool set_pin_interrupt(uint8_t pin, bool enabled);

  // For advanced usage: setup/read rotary encoders
  void    setup_rotary_encoder(uint8_t channel, uint8_t pin_a, uint8_t pin_b, uint8_t pin_c = 0, bool count_microsteps = true);
  int16_t read_rotary_encoder(uint8_t channel);
  void    clear_rotary_encoder(uint8_t channel);

private:
  // Low-level helpers
  uint8_t check_reset();
  void    wait_for_flash();

  uint8_t get_bit(uint8_t reg, uint8_t bit);
  void    set_bit(uint8_t reg, uint8_t bit);
  void    clr_bit(uint8_t reg, uint8_t bit);
  void    set_bits(uint8_t reg, uint8_t bits);
  void    clr_bits(uint8_t reg, uint8_t bits);
  void    change_bit(uint8_t reg, uint8_t bit, bool state);

  uint32_t board_millis() { return (uint32_t)millis(); }

private:
  PimoroniI2C *i2c;
  uint8_t address;
  uint8_t interruptPin;
  bool    pinSwapOut;
  uint32_t timeout;
  bool debug;
  float vref;

  // For rotary encoders
  int16_t encoder_offset[4];
  int16_t encoder_last[4];

  // The original code had a "Pin" struct array describing each pin's capabilities.
  // We'll replicate it thoroughly so that pin mode calls do the correct register changes.
  struct Pin {
    // The MS51 pin can be typed as IO, PWM, ADC, or ADC_OR_PWM
    enum IOType {
      TYPE_IO       = 0x01,
      TYPE_PWM      = 0x02,
      TYPE_ADC      = 0x04,
      TYPE_ADC_OR_PWM = (TYPE_ADC | TYPE_PWM)
    };
    IOType  type;
    uint8_t mode;

    uint8_t port;  // which port (0,1,3)
    uint8_t pin;   // which bit within that port
    uint8_t adc_channel;
    uint8_t pwm_channel;

    uint8_t reg_m1;
    uint8_t reg_m2;
    uint8_t reg_p;
    uint8_t reg_ps;           // Schmitt trigger config
    uint8_t reg_int_mask_p;   // INT_MASK_Px
    uint8_t reg_io_pwm;       // Which register toggles PWM enable bits
    uint8_t reg_pwml;         // PWML
    uint8_t reg_pwmh;         // PWMH

    bool mode_supported(uint8_t desired_mode);
    void set_mode(uint8_t new_mode);
    IOType get_type();
    uint8_t get_mode();

    // Constructors for convenience
    static Pin io(uint8_t port, uint8_t pin);
    static Pin pwm(uint8_t port, uint8_t pin, uint8_t channel, uint8_t reg_iopwm);
    static Pin adc(uint8_t port, uint8_t pin, uint8_t channel);
    static Pin adc_or_pwm(uint8_t port, uint8_t pin, uint8_t adc_channel, uint8_t pwm_channel, uint8_t reg_iopwm);
  };

  Pin pins[NUM_PINS];
};
