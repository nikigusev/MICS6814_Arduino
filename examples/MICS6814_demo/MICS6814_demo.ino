#include <Arduino.h>
#include "PimoroniI2C.h"
#include "IOExpander.h"
#include "MICS6814.h"

/*
 * Demonstration of fully replicated MICS6814 + IOExpander usage
 * Wiring on an Arduino UNO:
 *  SDA (A4) -> breakout SDA
 *  SCL (A5) -> breakout SCL
 *  5V (or 3.3V) -> breakout VIN  (Board is 3-5V tolerant)
 *  GND       -> breakout GND
 *
 * The default breakout address is 0x19.
 */

static const uint8_t MICS6814_ADDRESS = 0x19;   // default
static const bool    DEBUG            = true;

// PimoroniI2C object
PimoroniI2C pimoroni_i2c(Wire, 100000); // 100kHz

// IOExpander
IOExpander ioe(&pimoroni_i2c, MICS6814_ADDRESS, IOExpander::PIN_UNUSED, false, 1000, DEBUG);

// Create the MICS6814 sensor object
MICS6814 mics6814(ioe);

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  Serial.println("MICS6814 Gas Sensor Demo");

  if(!mics6814.init()) {
    Serial.println("MICS6814 init failed!");
    while(true) { delay(1000); }
  }

  // Turn on heater
  mics6814.set_heater(true);

  // Set LED to blue
  mics6814.set_led(0, 0, 255);
}

void loop() {
  // Read the sensor
  MICS6814::Reading reading = mics6814.read_all();
  Serial.print("Oxidising = ");
  Serial.print(reading.oxidising);
  Serial.print(" ohms, Reducing = ");
  Serial.print(reading.reducing);
  Serial.print(" ohms, NH3 = ");
  Serial.print(reading.nh3);
  Serial.println(" ohms");

  delay(1000);
}
