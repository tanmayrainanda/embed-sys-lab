# 🌟 Lab 3: Sensor Implementation 🚀

## Objective 🎯
This lab session focuses on exploring microcontrollers with varying capabilities and architectures (STM32 Nucleo-64, Arduino Uno, and ESP32 WROOM) using the Arduino IDE. 
You will understand how to interface with common sensors, benchmark their performance, and analyze the differences in behavior across different microcontrollers.

![image](https://github.com/user-attachments/assets/9cb32cbd-9cdc-4bcf-a5ad-992c69558af9)

## Key Learning Outcomes 📚
- Understand the differences in architecture and performance between STM32, Arduino, and ESP32.
- Interface and collect data from DHT11 (temperature and humidity sensor), MPU6050 (IMU sensor) using I2C, and a 10K variable potentiometer (analog sensor).
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
  - MPU6050 (Accelerometer/Gyroscope)
  - 10K Variable Potentiometer
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
   - Install libraries for DHT11 (`DHT sensor library`) and MPU6050 (`Adafruit MPU6050`).

### 2. **STM32 Nucleo-64 🟡**
   - Install STM32 drivers from [STMicroelectronics](https://www.st.com/en/development-tools/stsw-link009.html).
   - In the Arduino IDE, go to **Boards Manager** and install **STM32 Boards**.
   - Refer to the Lab-1 folder in the repo for relevant setup resources.
   - Install necessary libraries for DHT11 and MPU6050.

### 3. **ESP32 WROOM 🔵**
   - Install the ESP32 Arduino Core from **Boards Manager**.
   - Select **ESP32 Dev Module**.
     
     ![image](https://github.com/user-attachments/assets/a9c19673-5e50-4e27-83b9-30deab81ef96)
     
   - Install libraries for DHT11 and MPU6050.
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

## Note: The code remains the same just refer the datasheets for correct pin configurations

### 1. **Blink LED (Basic Test) 💡**
This activity verifies the successful configuration of each board.

#### Code (Common for All Boards)
```cpp
#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
```

**Expected Outcome:**
- The onboard LED should blink on all three boards.

### 2. **Interfacing DHT11 (Temperature & Humidity Sensor) 🌡️**
#### Code
```cpp
#include "DHT.h"
#define DHTPIN 2  // Connect DHT11 to pin D2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" °C");
  Serial.print(" Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  delay(2000);
}
```

### 3. **Interfacing MPU6050 Using I2C 📐**
#### Code
```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.println();
  delay(500);
}
```

### 4. **Interfacing 10K Variable Potentiometer (Analog Input) 🎚️**
#### Code
```cpp
#define POT_PIN A0

void setup() {
  Serial.begin(9600);
  pinMode(POT_PIN, INPUT);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);
  delay(500);
}
```

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
