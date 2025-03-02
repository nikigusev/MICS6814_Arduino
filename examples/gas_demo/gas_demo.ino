/*
  - Continuously read the MICS6814 gas sensor channels
  - Print the readings (oxidising, reducing, NH3) every second

  Wiring on Arduino UNO:
    SDA (A4) -> breakout SDA
    SCL (A5) -> breakout SCL
    5V       -> breakout VIN  (3-5V tolerant)
    GND      -> breakout GND

*/

#include <Arduino.h>
#include "PimoroniI2C.h"
#include "IOExpander.h"
#include "MICS6814.h"

// Default I2C address for recent MICS6814 breakouts is 0x19 (older ones use 0x18).
static const uint8_t MICS6814_ADDRESS = 0x19;

// Create I2C interface
PimoroniI2C pimoroni_i2c(Wire, 100000);

// Create IOExpander (no interrupt pin used here, pass PIN_UNUSED)
IOExpander ioe(&pimoroni_i2c, MICS6814_ADDRESS, IOExpander::PIN_UNUSED);

// Create MICS6814 sensor object
MICS6814 mics(ioe);

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }

  Serial.println("gas_demo.ino - Continuously read MICS6814 gas sensor...");

  // Initialize the sensor
  if(!mics.init()) {
    Serial.println("MICS6814 init failed! Check wiring/address.");
    while(true) { delay(1000); }
  }

  // Turn on heater
  mics.set_heater(true);

  // Optional: Set the LED to indicate the sensor is active (blue)
  mics.set_led(0, 0, 255);

  // A brief delay for sensor warm-up (optional short wait)
  delay(500);
}

void loop() {
  // Read all channels
  MICS6814::Reading reading = mics.read_all();

  // Print them
  Serial.print("Oxidising = ");
  Serial.print(reading.oxidising);
  Serial.print(" ohms,  Reducing = ");
  Serial.print(reading.reducing);
  Serial.print(" ohms,  NH3 = ");
  Serial.print(reading.nh3);
  Serial.println(" ohms");

  // Wait 1 second, then repeat
  delay(1000);
}
