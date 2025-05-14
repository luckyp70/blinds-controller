<!-- Project Badges -->

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4%2B-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

# ESP32 Blinds Controller

A smart window blinds controller that uses ESP32 microcontroller with Zigbee connectivity to automate window blinds/shades operation.

## Overview

This project implements a window blinds controller using ESP32 that can control up to two motors for automated window coverings. The controller integrates with Zigbee networks to provide wireless control capabilities and supports physical button controls for manual operation. It is designed for easy integration with Home Automation platforms (e.g., Home Assistant, Zigbee2MQTT, SmartThings) and supports OTA firmware updates and calibration.

## Features

- **Dual Motor Control**: Can control two separate motors for up/down movement
- **Physical Button Controls**: Four buttons for manual operation (up/down for each blind)
- **Zigbee Connectivity**: Integrates with home automation systems using Zigbee protocol (HA Window Covering cluster)
- **Safety Timeout**: Automatic motor cutoff after 15 seconds to prevent overheating
- **Window Covering Zigbee Cluster**: Implements the standard Zigbee HA Window Covering cluster (supports open/close/stop/position)
- **OTA Firmware Upgrade**: Supports Zigbee OTA cluster for remote firmware updates
- **Calibration & Persistence**: Stores calibration data in NVS for accurate movement durations
- **LED Feedback**: Uses onboard RGB LED for status and calibration feedback

## Hardware Requirements

- ESP32 development board (ESP32-C6 recommended)
- Motor driver (H-bridge) for controlling DC motors, like the DRV8871
- 2 DC motors for blinds operation
- 4 buttons for manual control
- Power supply appropriate for your motors and ESP32
- Blinds/shade mechanism compatible with the motors
- (Optional) RGB LED for feedback

### GPIO Pin Assignments

| Component      | GPIO Pin | Description                                       |
| -------------- | -------- | ------------------------------------------------- |
| Motor 1 IN1    | 4        | Motor 1 control pin 1 for DRV8871 H-Bridge driver |
| Motor 1 IN2    | 5        | Motor 1 control pin 2 for DRV8871 H-Bridge driver |
| Motor 2 IN1    | 6        | Motor 2 control pin 1 for DRV8871 H-Bridge driver |
| Motor 2 IN2    | 7        | Motor 2 control pin 2 for DRV8871 H-Bridge driver |
| Button T1 UP   | 11       | Blind 1 up button                                 |
| Button T1 DOWN | 10       | Blind 1 down button                               |
| Button T2 UP   | 15       | Blind 2 up button                                 |
| Button T2 DOWN | 14       | Blind 2 down button                               |
| RGB LED        | (config) | Optional, see mcu.c for pin assignment            |
| Motor 1 ADC    | 8        | Motor 1 current sense input (ADC, see motors.c)   |
| Motor 2 ADC    | 9        | Motor 2 current sense input (ADC, see motors.c)   |

## Software Requirements

- ESP-IDF v5.4 or later
- ESP-Zigbee-SDK (included as managed components)
- FreeRTOS (bundled with ESP-IDF)

## Building the Project

1. Install ESP-IDF following the [official installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
2. Clone this repository:
   ```
   git clone https://github.com/yourusername/esp32-blinds-controller.git
   cd esp32-blinds-controller
   ```
3. Configure the project (optional):

   ```
   idf.py menuconfig
   ```

   Navigate to "Blinds Controller Configuration" to customize hardware settings.

4. Build the project:

   ```
   idf.py build
   ```

5. Flash to your ESP32:

   ```
   idf.py -p PORT flash
   ```

   Replace PORT with your ESP32 serial port (e.g., /dev/ttyUSB0 on Linux, COM3 on Windows)

6. Monitor the output (optional):
   ```
   idf.py -p PORT monitor
   ```

## Zigbee Configuration

The controller is configured as a Zigbee End Device by default, implementing the Window Covering device profile. It can be paired with any Zigbee coordinator (like Home Assistant with Zigbee2MQTT, Samsung SmartThings, etc.).

- **Pairing**: Put your Zigbee coordinator in pairing mode and power up the controller. The device will join automatically.
- **Binding**: Use your Zigbee coordinator to bind the Window Covering cluster for remote control.
- **OTA**: The device supports Zigbee OTA firmware upgrades (see `zigbee_ota.c`).

To modify Zigbee settings, use `idf.py menuconfig` and navigate to "Blinds Controller Configuration" → "Zigbee Configuration".

## Calibration

- The controller supports automatic calibration of opening/closing durations for accurate position tracking.
- Calibration data is stored in NVS and is persistent across reboots.
- To recalibrate, reset calibration data in NVS or use a dedicated Zigbee command (if implemented).

## Usage

### Initial Setup

1. Connect the hardware according to the GPIO pin assignments
2. Flash the firmware to the ESP32
3. Power up the controller
4. Put your Zigbee coordinator in pairing mode
5. The controller will automatically join the Zigbee network

### Manual Control

- Press the UP buttons to raise the corresponding blind
- Press the DOWN buttons to lower the corresponding blind
- Press the same button again (or the opposite direction button) to stop movement

### Zigbee Control

The controller implements the Zigbee Home Automation Window Covering cluster, which supports the following commands:

- Up/Down/Stop
- Go to lift percentage
- Go to tilt percentage (if your blinds mechanism supports tilting)

## Project Structure

```
├── CMakeLists.txt          # Project-level CMake configuration
├── main/                   # Main application source code
│   ├── app_events.c/h      # Event system for inter-module communication
│   ├── blinds.c/h          # Blinds control logic and state
│   ├── buttons.c/h         # Button handling implementation
│   ├── motors.c/h          # Motor driver interface
│   ├── mcu.c/h             # MCU and LED utilities
│   ├── zigbee.c/h          # Zigbee stack integration
│   ├── zigbee_ota.c/h      # Zigbee OTA update support
│   └── zcl_utility/        # ZCL utility helpers
├── managed_components/     # Espressif Zigbee/LED libraries
├── components/             # Additional components (if any)
├── build/                  # Build output
├── README.md               # This file
└── docs/                   # Documentation and diagrams
```

## Main Software Modules

- **blinds.c/h**: Implements blind state, movement, calibration, and event handling
- **zigbee.c/h**: Handles Zigbee stack, device discovery, attribute reporting, and Zigbee events
- **motors.c/h**: Motor driver abstraction for up/down/stop
- **app_events.c/h**: Event system for decoupled module communication
- **zigbee_ota.c/h**: Zigbee OTA firmware update support
- **mcu.c/h**: MCU-level utilities (LED, etc.)

## License

This project is licensed under the MIT License - see the header comments in the source files for details.

## Authors

- Fortunato Pasqualone - Initial work - 2024–2025

## Troubleshooting

### Common Issues

- **Motors not moving**: Check wiring and power supply to motors
- **Buttons not responsive**: Verify GPIO connections and pull-up/pull-down settings
- **Cannot join Zigbee network**: Ensure coordinator is in pairing mode and within range
- **Uneven blind movement**: Adjust motor timeout value in motors.c (MOTOR_TIMEOUT_MS)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Acknowledgements

- [Espressif Zigbee SDK](https://github.com/espressif/esp-zigbee-sdk)
- [FreeRTOS](https://www.freertos.org/)
- [Zigbee2MQTT](https://www.zigbee2mqtt.io/)
- [Home Assistant](https://www.home-assistant.io/)

## Support / Contact

For questions, bug reports, or feature requests, please open an issue on GitHub or contact the author via [GitHub profile](https://github.com/yourusername).
