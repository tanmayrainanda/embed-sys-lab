# Obstacle Detection and Avoidance using STM32 Nucleo-64 Board 🚀

This project demonstrates how to use an **STM32 Nucleo-64 (STM32F103RBT6)** board with a **GY-521 IMU (MPU6050)** and **Ultrasonic Sensor (HC-SR04)** to detect and avoid obstacles. The system logs collisions based on distance measurements from the ultrasonic sensor and sudden acceleration changes detected by the IMU. The collision data is logged in a CSV file using a Python script.

![Project-2 (1)](https://github.com/user-attachments/assets/9890d642-4cf4-4789-86ff-b44a6c4264d4)

## Table of Contents

- [Project Overview 🌐](#project-overview)
- [Hardware Requirements 🔧](#hardware-requirements)
- [Software Requirements 📊](#software-requirements)
- [Setup and Wiring 🛠️](#setup-and-wiring)
- [Arduino Code 🔢](#arduino-code)
- [Python Logging Script 💻](#python-logging-script)
- [Running the Project ⏳](#running-the-project)
- [Resources and References 📖](#resources-and-references)
- [Future Improvements ✨](#future-improvements)

---

## Project Overview 🌐

This project integrates embedded systems and desktop applications. Data from the **GY-521 IMU** and **HC-SR04 Ultrasonic Sensor** is captured via the **STM32 Nucleo-64** board and processed using Arduino IDE. Collision events are detected and logged with timestamps in a CSV file.

## Hardware Requirements 🔧

- STM32 Nucleo-64 (STM32F103RBT6)
- GY-521 IMU (MPU6050)
- HC-SR04 Ultrasonic Sensor
- USB Cable for Nucleo Board
- Jumper Wires
- Windows/Linux/Mac PC

## Software Requirements 📊

- **Arduino IDE** (for uploading code to STM32 Nucleo)
  - Install STM32 board support via Arduino Boards Manager
  - Install **Adafruit MPU6050 Library**
- **Python 3.x** (for running the logging script)
  - Install required packages:
    ```bash
    pip install pyserial
    ```

## Setup and Wiring 🛠️

**GY-521 and HC-SR04 to STM32 Nucleo Wiring:**

| Component Pin | Nucleo Pin |
| ------------- | ---------- |
| GY-521 VCC    | 5V         |
| GY-521 GND    | GND        |
| GY-521 SDA    | PB7        |
| GY-521 SCL    | PB6        |
| HC-SR04 VCC   | 5V         |
| HC-SR04 GND   | GND        |
| HC-SR04 Trig  | 9          |
| HC-SR04 Echo  | 10         |

**Note:** Ensure the Nucleo board is powered through USB and recognized by your PC.

## Arduino Code 🔢

Below is the code for detecting obstacles and logging collision events:

```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  Serial.begin(9600);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Initialize Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("System Initialized. Monitoring for obstacles...");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Measure distance using ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration / 2.0) * 0.0343;  // Updated for more accurate distance calculation based on sound speed in air (34.3 microseconds per cm)

  // Detect sudden acceleration changes
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                               a.acceleration.y * a.acceleration.y +
                               a.acceleration.z * a.acceleration.z) - 1.0;  // Adjusted magnitude threshold by offsetting gravity (1g)

  if (distance < 3.0 && distance > 0.5) {
    if (accelMagnitude > 3.0) {
      Serial.print("Collision detected! Distance: ");
      Serial.print(distance);
      Serial.print(" cm, Accel Magnitude: ");
      Serial.println(accelMagnitude);
    } else {
      Serial.print("Obstacle detected at distance: ");
      Serial.print(distance);
      Serial.println(" cm, No collision.");
    }
  } else if (accelMagnitude < 0.1) {
    Serial.println("IMU stationary, no significant motion detected.");
  }

  delay(100);
}
```

Upload this code to the **Nucleo Board** using the Arduino IDE.

## Python Logging Script 💻

Below is the Python script for logging collision events to a CSV file:

```python
import serial
import csv
import time

# Serial port configuration (adjust to your port, e.g., "COM3" on Windows or "/dev/ttyUSB0" on Linux)
serial_port = 'COM3'  # Change this to match your system's serial port
baud_rate = 9600

# Open the serial port
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Connected to {serial_port} at {baud_rate} baud rate.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# Open a CSV file to log the data
csv_file_name = "collision_log.csv"

# Write headers to CSV
with open(csv_file_name, mode='w', newline='') as file:
    csv_writer = csv.writer(file)
    csv_writer.writerow(["Timestamp", "Event"])

print("Logging data to collision_log.csv...")

try:
    while True:
        # Read serial data
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Write the data to CSV
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                print(f"{timestamp}: {line}")
                with open(csv_file_name, mode='a', newline='') as file:
                    csv_writer = csv.writer(file)
                    csv_writer.writerow([timestamp, line])
except KeyboardInterrupt:
    print("Logging stopped by user.")
finally:
    ser.close()
    print("Serial port closed.")
```

## Running the Project ⏳

1. **Upload the Arduino Code:**

   - Connect the Nucleo board.
   - Upload the Arduino sketch.

2. **Run the Python Logging Script:**

   ```bash
   python your_script_name.py
   ```

3. Observe obstacle and collision events logged in the terminal and saved to `collision_log.csv`.

## Resources and References 📖

- [Arduino IDE](https://www.arduino.cc/en/software)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [STM32 Nucleo Documentation](https://www.st.com/en/evaluation-tools/nucleo-f103rb.html)

## Future Improvements ✨

- Implement wireless data transmission.
- Add advanced obstacle avoidance algorithms.
- Integrate additional sensors for more robust collision detection.



