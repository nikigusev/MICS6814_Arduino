# MICS6814 Arduino Library

An Arduino port for the [Pimoroni MICS6814 Breakout (PIM569)](https://shop.pimoroni.com/products/mics6814-gas-sensor-breakout). This library enables you to read qualitative changes in gas concentrations (reducing, oxidising, and NH3) and control the onboard RGB LED and sensor heater.

## Overview

The MICS6814 sensor integrates three semiconductor gas sensors into a single package. It is capable of detecting:
- **Reducing gases:** (e.g. carbon monoxide – CO)
- **Oxidising gases:** (e.g. nitrogen dioxide – NO₂)
- **Ammonia (NH₃)**

In addition, the sensor shows sensitivity to other compounds such as hydrogen, ethanol, and hydrocarbons. It is intended for qualitative measurements, allowing you to determine whether the gas levels are increasing or decreasing, rather than providing calibrated parts per million (ppm) values.

This breakout board features a Nuvoton MS51XB9AE microcontroller that handles I2C communications, analog-to-digital conversion (ADC), heater control, and PWM control of the RGB LED. The LED can be used to visually indicate sensor status or alert you to changes in gas concentrations.

## Features

- **MICS6814 3-in-1 Gas Sensor:** Qualitatively monitors reducing, oxidising, and NH₃ gas groups.
- **Nuvoton MS51XB9AE MCU:** Manages sensor ADC, LED PWM control, and heater functions.
- **RGB LED:** Provides visual status and alerts.
- **I2C Interface:** Default I2C address is `0x19`.
- **Voltage Flexibility:** Operates on 3V to 5V systems.
- **Reverse Polarity Protection:** Safeguards the board from incorrect power connections.
- **Raspberry Pi-Compatible Pinout:** Includes pins 1, 3, 5, 7, and 9 for easy integration.
- **Broad Ecosystem:** Compatible with existing Raspberry Pi Python libraries and Raspberry Pi Pico C++/MicroPython libraries.

## Installation

1. **Download the Library:**
   - Clone or download this repository and copy it into your Arduino libraries folder.

2. **Include the Library in Your Sketch:**
   ```cpp
   #include <MICS6814Wrapper.h>
   ```

3. **Initialize the Sensor:**
   ```cpp
   MICS6814Wrapper gasSensor;

   void setup() {
     Serial.begin(9600);
     if (!gasSensor.init()) {
       Serial.println("Failed to initialize the gas sensor!");
       while(1);
     }
   }
   ```

## Usage

### Reading Gas Sensor Values

You can read the qualitative resistance values (in ohms) corresponding to each gas group:

```cpp
void loop() {
  float reducing  = gasSensor.read_reducing();
  float nh3       = gasSensor.read_nh3();
  float oxidising = gasSensor.read_oxidising();

  Serial.print("Reducing: ");
  Serial.print(reducing);
  Serial.print(" ohms, NH3: ");
  Serial.print(nh3);
  Serial.print(" ohms, Oxidising: ");
  Serial.print(oxidising);
  Serial.println(" ohms");

  delay(1000);
}
```

### Controlling the LED

Set the RGB LED color using 8-bit values:

```cpp
// Example: Set LED to blue
gasSensor.set_led(0, 0, 255);
```

### Heater Control

Control the sensor’s heater using the following methods:

```cpp
// Turn the heater on
gasSensor.set_heater(true);

// Disable the heater (sets pin to input)
gasSensor.disable_heater();
```

## Known Issues

**Blue LED Impact on Sensor Readings:**  
When the blue LED is activated (e.g. via `set_led(0, 0, 255)`), you may observe a significant change in the sensor's ADC reference voltage. This voltage sag (from approximately 3.11 V with the LED off to around 2.30 V with blue on) leads to notably different and lower computed resistance values. This issue is likely due to the way the blue LED is driven (its PWM load is applied with a “load” operation) and/or the current draw of the blue LED affecting the shared voltage rail.

*Workarounds include:*
- Minimizing or avoiding continuous blue LED usage if accurate sensor readings are critical.
- Adding proper decoupling or filtering on the power supply.
- Isolating the sensor’s power or reference voltage from the LED drive circuit.
- Experimenting with PWM brightness and frequency settings.

## Background

This library is a port of the Raspberry Pi Pico C++ library provided by Pimoroni, translated for Arduino. While Pimoroni offers excellent libraries for Python (for Raspberry Pi) and C++/MicroPython (for Raspberry Pi Pico), there was no official Arduino library available for the PIM569 Breakout. This library bridges that gap while maintaining much of the original functionality, including the essential IOExpander capabilities.

For more information about the MICS6814 sensor and related projects, check out:
- [Pimoroni MICS6814 Python Library](https://github.com/pimoroni/mics6814-python)
- [Pimoroni Pico Libraries](https://github.com/pimoroni/pimoroni-pico)
- [Getting Started with Enviro+ Tutorial](https://learn.pimoroni.com/enviro)

## License

This library is licensed under the [MIT License](LICENSE).

## Acknowledgements

- **Pimoroni:** For their original designs, libraries, and tutorials.
- **Community Contributors:** Your feedback and contributions help improve this library.
