# esp32-norbert

This is the firmware to support the rotating platform developed as part of ME380. This directory includes the application source code for use on an ESP32-based platform using the ESP-IDF API. The intention of this firmware is to use a controller with a host PC which is set to communicate with the ESP32 over a USB tether.

## Setup

Ensure to install the ESP-IDF API available from Espressif Inc. and once installed navigate to the appropriate repository and source the appropriate development environment.

```bash
cd ~/esp/esp-idf/
source export.sh
```

Navigate to the directory containing the firmware and the appropriate commands can be used to both configure, build, and flash the project. However, it is recommended to use the ESP-IDF VS Code extension provided and maintined by Espressif.

## Configuration

### Fix Errors

If at any time in the following directions if an error code is received, first run the following command first to clean the build environment which should fix most errors.

```bash
# Run in project directory```
idf.py clean
```

### Configure FreeRTOS and Application Settings

To configure FreeRTOS and other setup configuration of the project, run the following command in the project directory. Follow the directions that appear on-screen.

```bash
# Run in project directory
idf.py menuconfig
```