# 🌟 Lab 3: Sensor Implementation 🚀

## Objective 🎯
This lab session focuses on exploring microcontrollers with varying capabilities and architectures (STM32 Nucleo-64, Arduino Uno, and ESP32 WROOM) using the Arduino IDE. 
You will understand how to interface with common sensors, benchmark their performance, and analyze the differences in behavior across different microcontrollers.

   https://github.com/user-attachments/assets/e32407f9-6267-4fef-bb70-fbf6e3dce335

## Key Learning Outcomes 📚
- Understand the differences in architecture and performance between STM32, Arduino, and ESP32.
- Interface and collect data from DHT11 (temperature and humidity sensor), HC-SR04 (ultrasonic distance sensor), GY-521 (IMU sensor) using I2C, and 10 potentiometers for analog to digital conversion.
- Write and upload common code bases using Arduino IDE with relevant drivers and libraries.
- Benchmark performance metrics and learn best practices for sensor integration.

---

## Required Components 🧩
- **Microcontrollers:**
  - STM32 Nucleo-64 (STM32F103RBT6)
  - Arduino Uno
  - ESP32 WROOM
- **Sensors:**
  - DHT11 (Temperature and Humidity Sensor)
  - HC-SR04 (Ultrasonic Distance Sensor)
  - GY-521 (Accelerometer/Gyroscope)
  - 10 Potentiometers for Analog to Digital Conversion
- **Additional:**
  - Jumper wires
  - Breadboard
  - USB cables
  - Power supply if needed
  - Multimeters for precise utilization during various operations 

---

## Setup Instructions 🔧

### 1. **Arduino Nano 🟢**
   - Install drivers from the [Arduino official website](https://www.arduino.cc/en/Guide/Nano).
   - Select board as **Arduino Nano** in the Arduino IDE.
   - Install libraries for DHT11 (`DHT sensor library`) and GY-521 (`Adafruit MPU6050`).

### 2. **STM32 Nucleo-64 🟡**
   - Install STM32 drivers from [STMicroelectronics](https://www.st.com/en/development-tools/stsw-link009.html).
   - In the Arduino IDE, go to **Boards Manager** and install **STM32 Boards**.
   - Refer to the Lab-1 folder in the repo for relevant setup resources.
   - Install necessary libraries for DHT11 and GY-521.

### 3. **ESP32 WROOM 🔵**
   - Install the ESP32 Arduino Core from **Boards Manager**.
   - Select **ESP32 Dev Module**.
     
     ![image](https://github.com/user-attachments/assets/a9c19673-5e50-4e27-83b9-30deab81ef96)
     
   - Install libraries for DHT11 and GY-521.
   - Relevant setup resources:
     - [USB Driver for ESP32](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads) ⚡
     - [Official Setup Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) 📚
     - [LED Blink Example](https://circuits4you.com/2018/02/02/esp32-led-blink-example/) 💡
     - [ESP32 Setup Guide](https://samueladesola.medium.com/how-to-set-up-esp32-wroom-32-b2100060470c) 🛠️
     - [Getting Started with ESP WROOM](https://www.ee-diary.com/2024/06/getting-started-with-esp-wroom-32-led.html) ✨
   - Use these board preferences URLs in Arduino IDE:
     - https://dl.espressif.com/dl/package_esp32_index.json
     - http://arduino.esp8266.com/stable/package_esp8266com_index.json     
     - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

---

## Activities 🛠️

### Note: The code remains the same, refer to the respective folder for code samples to test and datasheets for correct pin configurations.

### 1. **Blink LED (Basic Test) 💡**
This activity verifies the successful configuration of each board.

**Expected Outcome:**
- The onboard LED should blink on all three boards.

### 2. **Interfacing DHT11 (Temperature & Humidity Sensor) 🌡️**
### 3. **Interfacing HC-SR04 (Ultrasonic Sensor) 📏**
### 4. **Interfacing GY-521 Using I2C 📐**
### 5. **Interfacing 10 Potentiometers (Analog Input) 🎚️**

---

## Performance Benchmarking 📊
After running the same code on all three boards:
1. **Power Consumption:** ⚡
   - Measure using an ammeter for each microcontroller.
2. **Response Time:** ⏱️
   - Record the time to read sensor values and print to the serial monitor.
3. **Memory Usage:** 💾
   - Note memory usage displayed in the Arduino IDE during code upload.

| **Aspect**        | **STM32 Nucleo-64** | **Arduino Nano** | **ESP32 WROOM** |
|------------------|--------------------|----------------|----------------|
| Clock Speed      | 72 MHz             | 16 MHz         | 240 MHz        |
| Power Consumption| Moderate            | Low            | High           |
| Peripheral Support| High               | Moderate       | High           |
| Performance      | High                | Low            | High           |
| Ease of Use      | Moderate            | Easy           | Moderate       |

---

## Conclusion 📝
By the end of this lab, you will have gained practical experience with three different microcontrollers, understood their architectural differences, and learned how to choose the right controller for various applications based on performance and resource constraints.
