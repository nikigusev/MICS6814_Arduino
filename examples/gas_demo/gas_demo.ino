#include "MICS6814Wrapper.h"

MICS6814Wrapper sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial monitor
  if (sensor.init()) {
    Serial.println("MICS6814 sensor initialized successfully.");
  } else {
    Serial.println("Sensor initialization failed.");
  }
  
  // Turn the heater ON
  sensor.set_heater(true);
  
  // Set the LED color to blue (R=0, G=0, B=255)
  sensor.set_led(0, 0, 255);
}

void loop() {
  MICS6814::Reading reading = sensor.read_all();
  Serial.print("Reducing: ");
  Serial.print(reading.reducing);
  Serial.print(" | NH3: ");
  Serial.print(reading.nh3);
  Serial.print(" | Oxidising: ");
  Serial.print(reading.oxidising);
  Serial.print(" | Vref: ");
  Serial.println(reading.ref);
  delay(2000);
}
