# MEI Aquaculture Farmer 🚤🐟

**MEI Aquaculture Farmer** is an IoT-enabled autonomous boat system designed to continuously monitor water quality metrics in aquaculture farmlands. It features a dual-microcontroller architecture running on an ESP32-S3 custom controller and an ESP32-C6 autonomous boat.

The system relies heavily on Real-Time Operating Systems (FreeRTOS) and provides seamless dual forms of communication via **ESP-NOW** and **LoRa**, ensuring no telemetry point is lost even if one subsystem briefly disconnects. 

## 🌟 Key Features

### ⛵ **The Autonomous Boat (ESP32-C6)**
- **Water Quality Telemetry:** Continuous live monitoring of **pH level**, **TDS (Total Dissolved Solids)**, **Temperature**, and **Turbidity / NTU**.
- **Auto Return-To-Home (RTH):** Integrated GPS combined with an MPU9250 Magnetic Compass allows the boat to automatically record its home GPS coordinate and return when the signal is engaged, or manually triggered from the controller.
- **Smart Power Monitoring:** Built-in INA219 sensor paired with an EMA (Exponential Moving Average) filter monitors battery life dynamically.

### 🎮 **The Smart Controller (ESP32-S3)**
- **Voice Control AI:** Equipped with an I2S digital microphone, the controller has integration for both:
  - **Local Edge AI:** (1 click) Trigger Edge Impulse inferencing directly on the ESP32.
  - **Gemini Cloud AI:** (2 clicks) Send custom recorded payload strings to Google Gemini API for highly intelligent execution mapping.
- **Advanced TFT Dashboard:** A vibrant, real-time updated ST7789 graphical UI featuring sensor cards, dynamic battery bars, compass visuals, and connectivity statuses.
- **Live HTML Dashboard:** Broadcasts a local Wi-Fi Hotspot ("MEI-Controller") with a fast and ultra-modern Web App interface (`http://192.168.4.1`) configured dynamically to read sensors. 
- **Telegram Alert Protocol:** Connects to standard Wi-Fi to push urgent real-time alerts to the farmer's Telegram when an anomaly occurs (e.g. Dangerously High pH, high temperature, network disconnect, or low battery).

## 🛠️ Hardware Requirements

**Controller Unit:**
- 1x ESP32-S3 WROOM
- 1x 1.54" or 2.0" ST7789 TFT Display (SPI)
- 1x Ra-02 LoRa 433MHz Module
- 1x I2S INMP441 Digital Microphone
- 1x MAX98357A I2S Amplifier
- 1x Analog Dual-axis Joystick

**Boat Unit:**
- 1x ESP32-C6
- Water Sensors: Analog pH Sensor, Analog TDS Sensor, Analog Turbidity Sensor, Dallas DS18B20 Temp Sensor.
- Accelerometer & Compass: MPU9250 (AK8963)
- Battery Monitor: INA219 (I2C)
- Locating: GPS Module (TinyGPSPlus compatible)
- LoRa 433MHz Module (Backup channel)
- 2x/4x DC Motors via H-Bridge (L298N / TB6612FNG)

## 📡 Wireless Communication Stack

In an aquatic environment, signal degradation is a critical concern, so we designed a fault-tolerant mesh:
1. **ESP-NOW [Primary]:** Auto-syncing Wi-Fi channels to provide ultra-low latency, <40ms controller-to-boat joystick input updates.
2. **LoRa 433MHz [Secondary/Backup]:** Operates silently in the background on Spreading Factor 9. If ESP-NOW fails over far distances, LoRa automatically falls back to maintain critical manual controls and sensor updates.

## 🚀 Setup & Installation

1. Clone this repository directly into your Arduino IDE workspace.
2. Ensure you have the required libraries installed:
    - `Adafruit_GFX`, `Adafruit_ST7789`
    - `ArduinoJson`, `ESPAsyncWebServer`
    - `LoRa` by Sandeep Mistry
    - `DallasTemperature`, `TinyGPSPlus`, `MPU9250_WE`
3. Flash `ESP32S3.ino` to the **Controller**. Update `WIFI_SSID` / `WIFI_PASS` if you want automatic connection to your home network for Telegram capabilities.
4. Flash `ESP32C6.ino` to the **Boat**. Ensure MAC addresses match the array assigned in the scripts for ESP-NOW bridging.

## ⚠️ Notes for Production / Modifying
The **Web Server and Wi-Fi Scanning System** is heavily optimized under an **Asynchronous execution loop pattern** designed to temporarily sleep connections so it won't trigger **LwIP Core Panics** alongside audio transmission to Google Gemini AI.

*Developed with passion for Aquaculture 4.0!*
