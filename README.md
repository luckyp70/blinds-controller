<!-- Project Badges -->

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4%2B-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

# ESP32 Blinds Controller

A robust, Zigbee-enabled roller blinds controller for ESP32, supporting up to two independent blinds with position control, safety features, and seamless integration with modern home automation platforms (e.g., Home Assistant). Blinds can be operated seamlessly using either physical buttons or Zigbee commands.

## Overview

This firmware enables automated and manual control of two roller blinds using an ESP32 microcontroller with Zigbee support (such as ESP32-C6, ESP32-H2, or compatible). It features Zigbee Home Automation (HA) integration, position calibration, current sensing, OTA updates, and a modular, event-driven architecture. Designed for reliability and easy integration with platforms like Home Assistant and Zigbee2MQTT.

## Features

- **Dual Blind Control**: Independently control two roller blinds (up/down/stop/position)
- **Zigbee Home Automation**: Implements the Window Covering cluster for wireless integration
- **Precise Positioning**: 0–100% position control with calibration and persistent storage
- **Current Sensing**: ADC-based motor current monitoring for safety and diagnostics
- **Physical Buttons**: Manual up/down/stop for each blind
- **Safety Timeout**: Automatic cutoff to prevent motor overheating
- **OTA Updates**: Zigbee OTA cluster for remote firmware upgrades
- **LED Feedback**: RGB LED that blinks green when the device is identified via Zigbee and blue when calibration data is stored in NVM.
- **Modular Codebase**: Event-driven, easily extensible architecture

## Hardware Requirements

- 24V DC tubular motors (e.g., Somfy SOMFY ROLL UP 28 WT for small blinds)
- ESP32 development board (ESP32-C6 recommended)
- 2x DC motors and H-bridge drivers (e.g., DRV8871 suitable for small motors)
- 4x push buttons (manual control; must be properly debounced, active low to GND)
- Power supply for motors and ESP32
- (Optional) RGB LED for feedback

> **Note:** Physical buttons should be properly debounced in hardware and are activated when pulled low (connected to GND).
>
> **Current Sensing:** A simple current sensing circuit can be implemented with a rail-to-rail op-amp (VCC to +3.3V and VDD to GND) to amplify the voltage across a shunt resistor placed between ground and the GND terminal of the H-bridge power input. The op-amp output connects to the ESP32 ADC input defined for the roller blind.

### GPIO Pin Assignments

GPIO pins can be configured via menuconfig:

```sh
idf.py menuconfig
```

The default GPIO assignments are as follows:

| Component      | GPIO Pin | Description                                     |
| -------------- | -------- | ----------------------------------------------- |
| Motor 1 IN1    | 4        | Motor 1 control pin 1 for H-Bridge driver       |
| Motor 1 IN2    | 5        | Motor 1 control pin 2 for H-Bridge driver       |
| Motor 2 IN1    | 6        | Motor 2 control pin 1 for H-Bridge driver       |
| Motor 2 IN2    | 7        | Motor 2 control pin 2 for H-Bridge driver       |
| Button T1 UP   | 11       | Blind 1 up button                               |
| Button T1 DOWN | 10       | Blind 1 down button                             |
| Button T2 UP   | 15       | Blind 2 up button                               |
| Button T2 DOWN | 14       | Blind 2 down button                             |
| RGB LED        | (config) | Optional, see mcu.c for pin assignment          |
| Motor 1 ADC    | 8        | Motor 1 current sense input (ADC, see motors.c) |
| Motor 2 ADC    | 9        | Motor 2 current sense input (ADC, see motors.c) |

## Software Requirements

- ESP-IDF v5.4 or later
- Espressif Zigbee SDK (managed_components)
- FreeRTOS (bundled with ESP-IDF)

## Project Structure

```
├── main/
│   ├── app_events.c/h      # Event system for inter-module communication
│   ├── blinds.c/h          # Roller blinds logic, calibration, and state
│   ├── buttons.c/h         # Button handling
│   ├── motors.c/h          # Motor driver and current sensing
│   ├── mcu.c/h             # MCU and RGB LED utilities
│   ├── zigbee.c/h          # Zigbee stack, attribute reporting, OTA
│   ├── zigbee_ota.c/h      # Zigbee OTA update support
│   └── zcl_utility/        # ZCL helpers
├── docs/                   # Documentation and diagrams (to be released)
├── README.md               # This file
├── sdkconfig               # Project configuration
└── CMakeLists.txt          # Project-level CMake config
```

## Quick Start

1. **Install ESP-IDF** ([guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html))
2. **Clone the repository**
   ```sh
   git clone https://github.com/luckyp70/blinds-controller.git
   cd blinds-controller
   ```
3. **Configure hardware and Zigbee settings**
   ```sh
   idf.py menuconfig
   ```
   Adjust GPIOs and Zigbee options under "Blinds Controller Configuration".
4. **Build and flash**
   ```sh
   idf.py build
   idf.py -p PORT flash
   idf.py -p PORT monitor
   ```
   Replace `PORT` with your ESP32 serial port (e.g., `/dev/ttyUSB0`).

## Zigbee Integration

- **Pairing**: Power up and put your Zigbee coordinator in pairing mode. The device will join automatically.
- **Home Automation**: Compatible with Home Assistant, Zigbee2MQTT, SmartThings, and other Zigbee HA platforms.
- **OTA**: Supports Zigbee OTA firmware upgrades (see `zigbee_ota.c`).
- **Binding**: Use your coordinator to bind the Window Covering cluster for remote control.

## Calibration & Usage

- **Calibration**: Automatic after first full open-close cycle; durations stored in NVS. Recalibrate by clearing NVS (see [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html)).
- **Manual Control**: Use buttons for up/down/stop per blind. To stop, press the same direction button again while the blind is moving.
- **Zigbee Control**: Supports open, close, stop, and set position commands.

## License

MIT License. See source files for details.

## Author

- Fortunato Pasqualone — 2024–2025

## Contributing

Contributions are welcome! Please fork, branch, and submit a pull request.
