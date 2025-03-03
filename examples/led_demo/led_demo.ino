#include "MICS6814Wrapper.h"

MICS6814Wrapper sensor(false, 0x19); // debug = false, default I2C address

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (sensor.init()) {
    Serial.println("LED control initialized successfully.");
  } else {
    Serial.println("LED control initialization failed.");
  }
  
  // Turn the heater OFF
  sensor.disable_heater();
  
  // Set LED brightness to 50% (value between 0.01 and 1.0)
  sensor.setBrightness(0.5f);
  
  // Set LED PWM frequency to 500 Hz using the new PWM control function.
  // This will configure the underlying IOExpander's PWM settings.
  uint16_t pwmPeriod = sensor.setLEDPWMFrequency(500.0f);
  Serial.print("LED PWM period set to: ");
  Serial.println(pwmPeriod);
}

void loop() {
  // Define an array of rainbow colors: Red, Orange, Yellow, Green, Blue, Indigo, Violet.
  uint8_t colors[][3] = {
    {255,   0,   0},    // Red
    {255, 127,   0},    // Orange
    {255, 255,   0},    // Yellow
    {  0, 255,   0},    // Green
    {  0,   0, 255},    // Blue
    { 75,   0, 130},    // Indigo
    {148,   0, 211}     // Violet
  };
  const uint8_t numColors = sizeof(colors) / sizeof(colors[0]);
  
  for (uint8_t i = 0; i < numColors; i++) {
    sensor.set_led(colors[i][0], colors[i][1], colors[i][2]);
    delay(1000);
  }
}
