# GY-521 IMU Data Visualization with STM32 Nucleo-64 Board and Arduino IDE 🚀

This project demonstrates how to use an **STM32 Nucleo-64 (STM32F103RBT6)** board with a **GY-521 IMU (MPU6050)** to read motion data and visualize it on a GUI built with **Pygame** in Python. You'll capture accelerometer and gyroscope data from the GY-521 and visualize the data by moving a rectangle on a blank space, simulating real-time motion tracking.

## Table of Contents
- [Project Overview 🌐](#project-overview)
- [Hardware Requirements 🔧](#hardware-requirements)
- [Software Requirements 📊](#software-requirements)
- [Setup and Wiring 🛠️](#setup-and-wiring)
- [Arduino Code 🔢](#arduino-code)
- [Python GUI Script 💻](#python-gui-script)
- [Running the Project ⏳](#running-the-project)
- [Resources and References 📖](#resources-and-references)
- [Future Improvements ✨](#future-improvements)

---

## Project Overview 🌐
This project demonstrates the integration of embedded systems and desktop applications. Data from the **GY-521 IMU** is captured via the **STM32 Nucleo-64** board and processed using Arduino IDE. The data is sent to the host computer over a serial connection, where a **Python Pygame** GUI interprets it to control a rectangle's position on the screen based on sensor readings.

## Hardware Requirements 🔧
- STM32 Nucleo-64 (STM32F103RBT6)
- GY-521 IMU (MPU6050)
- USB Cable for Nucleo Board
- Jumper Wires
- Windows/Linux/Mac PC

## Software Requirements 📊
- **Arduino IDE** (for uploading code to STM32 Nucleo)
  - Install STM32 board support via Arduino Boards Manager
  - Install **Adafruit MPU6050 Library**
- **Python 3.x** (for running the GUI)
  - Install required packages:
    ```bash
    pip install pyserial pygame
    ```

## Setup and Wiring 🛠️
**GY-521 to STM32 Nucleo Wiring:**
| GY-521 Pin | Nucleo Pin |
|------------|------------|
| VCC        | 3.3V       |
| GND        | GND        |
| SDA        | PB7        |
| SCL        | PB6        |

**Note:** Ensure the Nucleo board is powered through USB and recognized by your PC.

## Arduino Code 🔢
Below is the code for reading sensor data from the GY-521:
```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Calibrating MPU6050... Keep the device stationary.");
  calibrateMPU();
  Serial.println("Calibration complete.");
  Serial.println("Starting data transmission...");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration offsets
  float accelX = a.acceleration.x - accelXOffset;
  float accelY = a.acceleration.y - accelYOffset;
  float accelZ = a.acceleration.z - accelZOffset;
  float pitch = atan2(accelY, accelZ) * 180 / PI;
  float roll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  Serial.print("X: "); Serial.print(accelX);
  Serial.print(" Y: "); Serial.print(accelY);
  Serial.print(" Z: "); Serial.print(accelZ);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.println(roll);

  delay(100);
}

void calibrateMPU() {
  const int numReadings = 100;
  float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumAccelX += a.acceleration.x;
    sumAccelY += a.acceleration.y;
    sumAccelZ += a.acceleration.z - 9.81;
    delay(10);
  }
  accelXOffset = sumAccelX / numReadings;
  accelYOffset = sumAccelY / numReadings;
  accelZOffset = sumAccelZ / numReadings;
}
```
Upload this code to the **Nucleo Board** using the Arduino IDE.

## Python GUI Script 💻
Below is the Python script for reading data from the Nucleo board and visualizing it in a Pygame window:
```python
import serial
import pygame
import re

ser = serial.Serial('COM4', 9600)  # Update to your correct port
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("GY-521 Data Visualization")
clock = pygame.time.Clock()

rect_width, rect_height = 50, 50
rect_x, rect_y = width // 2, height // 2
white = (255, 255, 255)
blue = (0, 0, 255)

def parse_sensor_data(line):
    match = re.match(r'X: ([\d\.\-]+) Y: ([\d\.\-]+) Z: ([\d\.\-]+) Pitch: ([\d\.\-]+) Roll: ([\d\.\-]+)', line)
    if match:
        return [float(x) for x in match.groups()]
    return None

running = True
while running:
    screen.fill(white)

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        data = parse_sensor_data(line)
        if data:
            x, y, z, pitch, roll = data
            rect_x += int(x * 5)
            rect_y -= int(y * 5)
            rect_x = max(0, min(width - rect_width, rect_x))
            rect_y = max(0, min(height - rect_height, rect_y))

    pygame.draw.rect(screen, blue, (rect_x, rect_y, rect_width, rect_height))
    pygame.display.flip()
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

ser.close()
pygame.quit()
```

## Running the Project ⏳
1. **Upload the Arduino Code:**
   - Connect the Nucleo board.
   - Upload the Arduino sketch.

2. **Run the Python GUI Script:**
   ```bash
   python your_script_name.py
   ```
3. Move the GY-521 sensor and observe the rectangle movement on the screen.

## Resources and References 📖
- [Arduino IDE](https://www.arduino.cc/en/software)
- [STM32 Nucleo Documentation](https://www.st.com/en/evaluation-tools/nucleo-f103rb.html)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [Pygame Documentation](https://www.pygame.org/docs/)

## Future Improvements ✨
- Add 3D visualization for better motion representation.
- Integrate machine learning for motion pattern recognition.
- Optimize serial data parsing for faster performance.

Happy Coding! 🚀

