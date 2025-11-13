# LilyGo-T-SIM7080G IoT Tracker

This project is an advanced IoT tracking device based on the LilyGo T-SIM7080G board. It integrates motion sensing, GPS, and multiple connectivity options (WiFi and Cellular) to create a versatile asset tracker for car and serves mainly to turn on dash cam to recrd events if car has been shocked.
The device is highly configurable via a web interface and is optimized for low-power operation.

## Key Features

- **Power Management:**
  - Utilizes the AXP2101 Power Management Unit (PMU).
  - Supports deep sleep modes to conserve battery.
  - Multiple wakeup sources: motion (IMU), timer, and power button/VBUS connection.
  - Battery level monitoring and automatic shutdown on critical battery levels.
  - TODO : add output to retain a relay of phone charger in the car to be on for specific time ( ex 5 minute ) after cars is turned off

- **Motion Detection:**
  - Uses an MPU6500 IMU with its Digital Motion Processor (DMP) to detect motion.
  - Wake-on-Motion (WOM) functionality to wake the device from deep sleep.
  - Configurable motion sensitivity and thresholds via the web interface.
  - DONE : rework the init of Wake-on-motion initialization of the mpu to optimize porpoer detection and power management 
  - DONE : Enabled cycle wake up and reduced accelerometer output rate in low power mode.

- **Connectivity:**
  - **WiFi:** Can operate in both Station (STA) and Access Point (AP) modes.
  - **Cellular:** Integrates the SIM7080G modem for GPRS/NB-IoT connectivity.
  - **MQTT:** Publishes data (e.g., logs, GPS location) to an MQTT broker.
  - **Over-the-Air (OTA) Updates:** Supports firmware updates via WiFi.

- **Sleep / Periodic Wakeup**
  - support waking up each < 1h - configurable >  to take snapshot ;
    wake up -> turn on cam , sleep for 10s --> wake up & turn off camera , return to secure mode ( motion detection ) 

- **BLEUTOOTH / BLE:** 
  - TODO: broadcast BLE periodically ( maybe each 1m ? very low power needed) ( got sleep with time & flag that we are waking up for BLE ? or other approach)
  - TODO: pair with phone via web server to enable bleutooh page and its config ( start pairing process , removed paried device, reset , other config)
  - TODO: when car started -> start bleutooth and try connect to phone, upon connecting to phone :
          sync time if possible ? develop android app ? 
          when Vbus ( car running ) & phone connected and ready to receive file -> send logs from files stored in file system ? or notifications maybe ? 

- **GPS Tracking:**
  - Onboard GPS functionality provided by the SIM7080G modem.
  - Periodically acquires and can report location data (optional) .
  - TODO : when motion is detected without car started within 5m, acquire GPS position, timestamp of wake time or shock time, and store it in log
  - TODO : possibly send the gps info and timestamp to mqtt message over mqtts , or post HTTPS request from the Cellular modem when possible.

- **Configuration:**
  - Provides a web server (in AP mode or STA mode) for configuration.
  - Settings are stored in Non-Volatile Storage (NVS).
  - Configurable parameters include:
    - WiFi and MQTT credentials.
    - IMU motion detection thresholds.
    - System timing for sleep and WiFi timeouts.
    - iBeacon UUID.
    - Web server credentials.

- **Peripherals:**
  - **SD Card:** Support for data logging and file storage.
  - **Status LEDs:** Uses FastLED library to control WS2811 LEDs for status indication.
  - **Real-Time Clock (RTC):** External DS1302 for timekeeping (optional).

## Hardware Components
- **Main Board:** LilyGo T-SIM7080G
- **PMU:** AXP2101
- **IMU:** MPU6500
- **RTC:** DS1302 (optional)
- **Storage:** MicroSD Card Slot
- **LED:** WS2811 RGB LED (2 leds)

## Software Architecture

The firmware is built on the Arduino framework for ESP32. The codebase is modular, with distinct components for handling different hardware features.

- `src/main.cpp`: The main application entry point. It contains the primary `setup()` and `loop()` functions, managing the device's overall state (e.g., sleeping, waking, connecting to WiFi).
- `src/power/`: Manages the AXP2101 PMU, handling battery charging, power domain control, deep sleep, and wakeup reasons.
- `src/imu6500/`: Implements the motion detection logic using the MPU6500's DMP. It calibrates the sensor and provides a simple interface to check for motion events.
- `src/modem/`: Controls the SIM7080G modem. This module is responsible for initializing the modem, managing the cellular connection (GPRS), and acquiring GPS data.
- `src/wifi/`: Handles all WiFi-related tasks, including connecting as a client (STA), creating an access point (AP), running the MQTT client, and managing OTA updates.
- `src/web_server/`: Implements a web-based configuration portal. It allows the user to change various system settings, which are saved to persistent storage.
- `src/sdcard/`: Provides functions to initialize and manage the SD card.
- `src/fast_led/`: A thread-safe controller for the status LEDs, supporting solid, blinking, and fading patterns.
- `src/rtc/`: Manages the external DS1302 Real-Time Clock.
- `src/camControl.cpp`: Simple functions to power the camera on and off.
- `src/pins.hpp`: Centralized header for all GPIO pin definitions.

## Getting Started

1.  **ESP IDF:** This project is set up to be built with ESP IDF. Open the project in an IDE like VSCode with the extension.
2.  **Configuration:**
    - When the device first starts or cannot connect to a known WiFi network, it will enter Access Point (AP) mode.
    - Connect to the WiFi network created by the device (the default is "ap_ssid").
    - Navigate to the device's IP address (usually 192.168.4.1) in a web browser.
    - Use the web interface to configure your WiFi credentials, MQTT broker details, and other settings.
3.  **Build and Upload:** Use ESP IDF to build and upload the firmware to the T-SIM7080G board.