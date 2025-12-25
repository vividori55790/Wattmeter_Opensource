
---

# KW Watt-Meter: Advanced Dual-MCU Power Monitoring System

An advanced, open-source power monitoring and protection system built with a dual-MCU architecture. This project provides real-time electrical analysis, including RMS values, harmonic distortion, and fuzzy-logic-based safety features, all accessible through an interactive touch UI and remote web configuration.

---

## 🚀 Key Features

### 1. High-Precision Power Analysis

* **Comprehensive Metrics**: Real-time monitoring of Voltage (V), Current (I), Real Power (P), Reactive Power (Q), Apparent Power (S), and Power Factor (PF).
* **Harmonic Analysis**: Performs FFT (Fast Fourier Transform) to analyze up to the 15th harmonic order for both voltage and current.
* **Triple-Channel Current Sensing**: Monitors a main line plus two individual loads (Load 1 and Load 2) simultaneously.

### 2. Fuzzy Logic Protection System

* **Predictive Safety**: Uses a Fuzzy Logic controller to evaluate risk based on real power levels and the rate of power change ().
* **Intelligent Tripping**: Implements multi-level safety responses, from warning alerts to instant relay cut-offs for critical surges or short circuits.
* **Inrush Current Tolerance**: Smart logic distinguishes between dangerous surges and normal motor/device start-up currents.

### 3. Visual & Interactive Interface

* **TFT Touch UI**: A 320x240 color display provides a dashboard for main metrics, a phasor diagram, and a harmonic bar graph.
* **Real-time Oscilloscope**: Integrated waveform viewer with auto-ranging and trigger modes (Continuous/Single).
* **Theming**: Supports both Dark and Light modes for optimized visibility.

### 4. Connectivity & Remote Config

* **IoT Ready**: Periodically uploads data to ThingSpeak for remote monitoring.
* **Web Config Portal**: Features a "WiFi Config Mode" that launches a SoftAP (SSID: `Wattmeter_Setup`) and a built-in web server for easy network setup and API key management.

---

## 🛠️ Hardware Requirements

| Component | Specification |
| --- | --- |
| **MCU 1 (Processor)** | Arduino R4 (Renesas-based) |
| **MCU 2 (Controller)** | Arduino-compatible with high ADC resolution |
| **Display** | ILI9341 320x240 TFT LCD |
| **Touch Panel** | XPT2046 Resistive Touch |
| **Connectivity** | ESP-01 (WiFi Module) |
| **Sensors** | AC Voltage Sensor (A3) & Current Sensors (A2, A4, A5) |
| **Protection** | 2-Channel Relay Module |

---

## 📂 Software Architecture

* **Processor (MCU 1)**: Dedicated to high-speed sampling, FFT computation, and Fuzzy Logic evaluation.
* **Controller (MCU 2)**: Manages the Graphical User Interface (GUI), touch input, and WiFi communication.
* **Communication**: The two MCUs exchange JSON-formatted data packets via Hardware Serial at 115200 bps.

---

## 🔧 Installation & Setup

### 1. Library Dependencies

Ensure you have the following libraries installed in your Arduino IDE:

* `arduinoFFT` (v2.x)
* `eFLL` (Embedded Fuzzy Logic Library)
* `ArduinoJson`
* `Adafruit_ILI9341` & `Adafruit_GFX`
* `XPT2046_Touchscreen`

### 2. Configuration

1. Upload `Wattmeter_MCU1_Processor.ino` to the Arduino R4.
2. Upload the `MCU2_Controller` files to your display controller.
3. On first boot, navigate to **Settings > Network > Web Config Mode** to connect the device to your local WiFi.

### 3. Calibration

The system supports both manual multiplier adjustment and an automatic calibration routine through the UI to ensure measurement accuracy.

---

## 👥 Team

* **Park Raewon** - Team Leader, Processot.ino
* **Park Yeseong** - Development, Controller.ino
* **Joo Hwijae** - Development, Hardware

---

## ⚠️ Disclaimer

This project involves measuring high-voltage AC electricity. Always use extreme caution. The authors are not responsible for any damage or injury caused by the use of this software or hardware.

---

Would you like me to add a **detailed technical section explaining the Fuzzy Logic membership functions** (, , etc.) or a **wiring guide** based on the pin definitions in the code?
