# 🌟 Lab 1: Introduction to Embedded Systems & Controllers
## 🚀 Task: LED Blinking Challenge

### 🎯 Objective
Implement a basic embedded system by blinking an LED using STM32 Nucleo and Arduino IDE.

![STM32F103RB](https://static.rapidonline.com/catalogueimages/product/75/07/s75-0774p02wc.jpg)

## 🛠️ Required Components

- 🖥️ **STM32 Nucleo Board** (e.g., STM32F103, STM32F401)
- 🔌 **USB Cable** (for power and programming)
- 💡 **LED** (if not using an onboard LED)
- 🔩 **Resistor** (330Ω if using an external LED)
- 🔗 **Breadboard & Jumper Wires** (if required)

## 📌 Procedure

### 📥 Install Required Software

- 📥 Download and install **Arduino IDE**.
- 🔧 Install **STM32 board support package** in Arduino IDE via Board Manager.
- Via this link to -> files-> Prefferences -> "Additional board Manager URLs": https://raw.githubusercontent.com/stm32duino/BoardManagerFiles/refs/heads/main/package_stmicroelectronics_index.json

  ![preferences](https://engineerworkshop.com/content/images/2020/01/javaw_3252VVQ7TU.png)

- Then make these configurations below shown:
  ![Arduino-IDE-config-nucleo-64](https://github.com/855princekumar/embed-sys-lab/blob/e0ad30883337065835d61a01800072dec2efedaa/softwares/Arduino-IDE-config-nucleo-64.png)

- 🔄 Install necessary drivers for STM32 from ST's official website/ Attached in the software folder in stm32F103RB-nucleo-driver.

### 🔗 Connect the STM32 Nucleo Board

- 🔌 Plug the board into your PC using a USB cable.
- 🖥️ Open **Arduino IDE** and ensure the board is detected.
- ✅ Select the appropriate STM32 board and port in **Tools → Board** and **Tools → Port**.

### ✍️ Write and Upload Code

Two versions of the LED blinking program are available in the **board-blink** folder:

1. ⚡ **Basic LED Blinking:** A simple program that toggles the LED at a fixed interval using `delay()`.
2. 🎛️ **Button-Controlled LED Speed:** A more optimized version that allows changing the LED blink speed using the onboard button, implemented using `millis()` for non-blocking execution.

### 🔄 Difference Between the Two Versions

- 🕒 **Basic Version (`delay()`)**: 
  - Uses `delay()`, which **halts execution** for a fixed duration. 
  - Simple but inefficient, as it **blocks** other tasks while waiting.
- 🚀 **Optimized Version (`millis()`)**:
  - Replaces `delay()` with `millis()` for **non-blocking execution**.
  - Allows other processes to run **simultaneously**.
  - Preferred for **real-world applications** where multitasking is needed.

### 🌍 Real-Life Example

Using `millis()` is beneficial in scenarios like:
- 🚦 **Traffic light control**: Different lights must operate asynchronously without blocking execution.
- 📡 **IoT devices**: Sensors need continuous data acquisition without interruptions.
- 🎛️ **Multitasking in embedded systems**: Enables handling multiple tasks like reading sensors while controlling an output.

## 🔍 Observe LED Blinking

- 💡 If using the onboard LED, it should blink at different speeds based on button presses.
- 🛠️ If using an external LED, ensure proper wiring and power connections.

## 🛠️ Troubleshooting

- ❌ **Board not detected?** Reinstall the **STM32 board support package** in Arduino IDE.
- 🔎 **LED not blinking?** Verify that the correct pin is selected for LED output.
- 🔄 **External LED issues?** Check wiring and power connections.

## 📚 References

- 🎥 [STM32 Nucleo LED Blink Tutorial - YouTube](https://youtube.com/playlist?list=PLr0mEvO7yBe6Ga7wJpRTZpxAYSvgWY0A-&si=GU9wgqeWRZXWg_fN)
- 🎥 [STM32 Nucleo Full with stmcubeIDE - YouTube](https://youtube.com/playlist?list=PLEBQazB0HUyRYuzfi4clXsKUSgorErmBv&si=xhcNUxrIFVgaGwq5)
- 🎥 [STM32 Nucleo Full with ArduinoIDE - YouTube](https://www.youtube.com/watch?v=FzxLPDNBqng&t)
- 📑 [Official STM32 Documentation](https://os.mbed.com/platforms/ST-Nucleo-F103RB/)
