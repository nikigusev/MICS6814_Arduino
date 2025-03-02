/*
  - Initialize the MICS6814 breakout
  - Adjust the LED brightness
  - Set an RGB color

  Wiring on Arduino UNO:
    SDA (A4) -> breakout SDA
    SCL (A5) -> breakout SCL
    5V       -> breakout VIN  (3-5V tolerant)
    GND      -> breakout GND

  This example doesn't read from the sensor. It just configures the LED.
  If you want to read the gas channels, see the "gas_demo.ino" example.
*/

#include <Arduino.h>
#include "PimoroniI2C.h"
#include "IOExpander.h"
#include "MICS6814.h"

// Default I2C address for MICS6814 breakout
static const uint8_t MICS6814_ADDRESS = 0x19;

// Create I2C interface
PimoroniI2C pimoroni_i2c(Wire, 100000);

// Create IOExpander
IOExpander ioe(&pimoroni_i2c, MICS6814_ADDRESS, IOExpander::PIN_UNUSED);

// Create MICS6814 sensor object
MICS6814 mics(ioe);

// You can adjust these to set the LED color
// or read them from the serial monitor in a more advanced version.
static const uint8_t R = 128;
static const uint8_t G = 64;
static const uint8_t B = 255;

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  Serial.println("led_demo.ino - Set the RGB LED on the MICS6814 breakout.");

  // Initialize sensor
  if(!mics.init()) {
    Serial.println("MICS6814 init failed! Check wiring/address.");
    while(true) { delay(1000); }
  }

  // Optionally adjust PWM period and brightness
  // By default, MICS6814::init() sets a period based on brightness.
  // If you want a very specific period:
  ioe.set_pwm_period(4096);  // from led.py
  mics.set_brightness(0.1f); // from led.py

  // Set LED color
  Serial.print("Setting LED color to (");
  Serial.print(R); Serial.print(", ");
  Serial.print(G); Serial.print(", ");
  Serial.print(B); Serial.println(").");

  mics.set_led(R, G, B);

  // No loop code needed; we can just hold here
}

void loop() {
  // The LED stays in the chosen color & brightness
  // You could read the sensor or do other tasks here...
}
