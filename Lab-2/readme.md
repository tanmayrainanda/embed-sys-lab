# 🌟 Lab 2: Bare Metal Programming on STM32F103RB-Nucleo

Welcome to the **Bare Metal Programming Lab**! In this lab, you will learn how to program the **STM32F103RB-Nucleo board** using **bare metal C** in the **Arduino IDE**. You will implement two use cases:
1. **LED Flashing Speed Control Using Delay** ⏳
2. **LED Flashing Speed Control Using Millis** ⏱️

   ![stm32](https://github.com/user-attachments/assets/f3f2674e-00b5-40ef-8e6d-065de55f69b3)

This lab builds on the first lab, where you used the Arduino IDE to program the onboard button and LED. Now, you will achieve the same functionality using **register-level programming** without relying on Arduino libraries.

---

## Table of Contents 📚
1. [Lab Objectives](#lab-objectives-)
2. [Prerequisites](#prerequisites-)
3. [STM32F103RB-Nucleo Board Overview](#stm32f103rb-nucleo-board-overview-)
4. [Bare Metal Programming Basics](#bare-metal-programming-basics-)
5. [Use Case 1: LED Flashing Speed Control Using Delay](#use-case-1-led-flashing-speed-control-using-delay-)
6. [Use Case 2: LED Flashing Speed Control Using Millis](#use-case-2-led-flashing-speed-control-using-millis-)
7. [Conclusion](#conclusion-)
8. [Additional Resources](#additional-resources-)

---

## Lab Objectives 🎯
- Learn **bare metal programming** on the STM32F103RB-Nucleo board.
- Understand **GPIO configuration**, **interrupts**, and **state machines**.
- Implement two use cases:
  1. Control LED flashing speed using a **delay-based approach**.
  2. Control LED flashing speed using a **millis-based approach**.
- Gain hands-on experience with **register-level programming** in the Arduino IDE.

---

## Prerequisites ✅
- **Hardware**:
  - STM32F103RB-Nucleo board.
  - USB cable for programming and power.
- **Software**:
  - Arduino IDE with STM32duino support (installed via STM32 Core)/ STM32Cube-IDE(reffer the folder with-cube-ide)
  - Add the below Board in the preference to setup the enviourment for STM controllers via: file-->Add_Prefereences
    
        http://dan.drown.org/stm32duino/package_STM32duino_index.json
        https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
        https://raw.githubusercontent.com/stm32duino/BoardManagerFiles/refs/heads/main/package_stmicroelectronics_index.json
       
       
  - STM32F1xx HAL library 
 
        https://github.com/STMicroelectronics/STM32CubeF1
        https://github.com/STMicroelectronics/stm32f1xx-hal-driver/tree/master
        https://github.com/stm32duino/Arduino_Core_STM32/blob/main/system/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h (Place this in the project folder after download)
    
  - Download the HAL & CMSIS Drivers from the official stm32 repo as required to be in every project folder as a dependency
  - Download the repo and just copy paste the 2 folder from STM32CubeF1/Drivers/------> CMSIS & STM32F1xx_HAL_Driver Drivers
    [Official-Driver-repo-link](https://github.com/STMicroelectronics/STM32CubeF1/tree/master/Drivers)
    
- **Knowledge**:
  - Basic understanding of C programming.
  - Familiarity with Arduino IDE and STM32 boards.

---

## STM32F103RB-Nucleo Board Overview 🖥️
The **STM32F103RB-Nucleo board** is a development board based on the **STM32F103RB microcontroller**. It features:
- **ARM Cortex-M3** core running at 72 MHz.
- **128 KB Flash** and **20 KB RAM**.
- Onboard **LED** (connected to PC13).
- Onboard **button** (connected to PC13 or another GPIO pin).
- Arduino-compatible headers for easy prototyping.

---

## Bare Metal Programming Basics ⚙️
Bare metal programming involves writing code that directly interacts with the hardware registers of the microcontroller, without relying on high-level libraries or frameworks. Key concepts include:
- **GPIO Configuration**: Setting up GPIO pins as inputs or outputs.
- **Clock Configuration**: Enabling clocks for peripherals.
- **Interrupts**: Handling external events using interrupts.
- **State Machines**: Implementing logic to manage system states.

---

## Use Case 1: LED Flashing Speed Control Using Delay ⏳

### Objective 🎯
Control the flashing speed of the onboard LED using the onboard button. The LED flashing speed should change based on the number of button presses:
- **1 press**: Delay = 50 ms.
- **2 presses**: Delay = 500 ms.
- **3 presses**: Delay = 1000 ms.

### Steps 🛠️
1. Configure the onboard LED (PA5) as an output.
2. Configure the onboard button (PA5 or another pin) as an input with an interrupt.
3. Use a state machine to track the number of button presses.
4. Change the LED flashing speed based on the state.

### Code Example (Basic OnBoard led blink - OK Tested) 💻
```c

#include "stm32f1xx.h"  // Include STM32 header for direct register access

// Improved delay function
void delay(int count) {
    for (volatile int i = 0; i < count; i++) {
        for (volatile int j = 0; j < 1000; j++) {
            __asm__("nop"); // No operation to prevent compiler optimization
        }
    }
}

int main(void) {
    // Enable GPIOA clock (for onboard LED on Nucleo F103RB)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configure PA5 as output (LED pin)
    GPIOA->CRL &= ~GPIO_CRL_MODE5;    // Clear MODE5 bits
    GPIOA->CRL &= ~GPIO_CRL_CNF5;     // Clear CNF5 bits
    GPIOA->CRL |= GPIO_CRL_MODE5_1;   // Set MODE5 to output (2 MHz)

    // Main loop
    while (1) {
        GPIOA->ODR |= GPIO_ODR_ODR5;  // Turn on LED (set PA5 high)
        delay(500);                   // Delay
        GPIOA->ODR &= ~GPIO_ODR_ODR5; // Turn off LED (set PA5 low)
        delay(500);                   // Delay
    }

    return 0; // This line is not needed in an infinite loop but added for completeness
}

```

### Code Example (Basic OnBoard led blink with delay change via button long press - OK Tested) 💻
```c

#include "stm32f1xx.h"  // Include STM32 header for direct register access

// Delay function for the LED
void delay(int count) {
    for (volatile int i = 0; i < count; i++) {
        for (volatile int j = 0; j < 1000; j++) {
            __asm__("nop"); // No operation to prevent compiler optimization
        }
    }
}

// Debounce function for PC13 button press
int debounce_button(void) {
    if (!(GPIOC->IDR & GPIO_IDR_IDR13)) {  // Button pressed (active low)
        delay(50);  // Debounce delay (50ms)
        if (!(GPIOC->IDR & GPIO_IDR_IDR13)) {  // Confirm button press
            return 1;
        }
    }
    return 0;
}

int main(void) {
    // Enable GPIOA and GPIOC clocks (for onboard LED and button)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

    // Configure PA5 as output (LED pin)
    GPIOA->CRL &= ~GPIO_CRL_MODE5;    // Clear MODE5 bits
    GPIOA->CRL &= ~GPIO_CRL_CNF5;     // Clear CNF5 bits
    GPIOA->CRL |= GPIO_CRL_MODE5_1;   // Set MODE5 to output (2 MHz)

    // Configure PC13 as input (button pin)
    GPIOC->CRH &= ~GPIO_CRH_MODE13;   // Clear MODE13 bits
    GPIOC->CRH &= ~GPIO_CRH_CNF13;    // Clear CNF13 bits
    GPIOC->CRH |= GPIO_CRH_CNF13_0;   // Set CNF13 to input floating

    int delay_time = 500;  // Default delay time (500ms)

    // Main loop
    while (1) {
        if (debounce_button()) {
            // Change delay time based on button press
            if (delay_time == 500) {
                delay_time = 1000;  // Change delay to 1000ms
            } else if (delay_time == 1000) {
                delay_time = 50;    // Change delay to 50ms
            } else {
                delay_time = 500;   // Change delay to 500ms
            }
            // Wait for button release
            while (!(GPIOC->IDR & GPIO_IDR_IDR13)) {
                delay(10);  // Small delay to prevent bouncing
            }
        }

        GPIOA->ODR |= GPIO_ODR_ODR5;  // Turn on LED (set PA5 high)
        delay(delay_time);            // Delay based on button press
        GPIOA->ODR &= ~GPIO_ODR_ODR5; // Turn off LED (set PA5 low)
        delay(delay_time);            // Delay based on button press
    }

    return 0;  // This line is not needed in an infinite loop but added for completeness
}

```

## Use Case 2: LED Flashing Speed Control Using Millis ⏱️

### Objective 🎯
Control the flashing speed of the onboard LED using the onboard button. Instead of using a delay function, use a **millis-based approach** to track time and toggle the LED.

### Steps 🛠️
1. Configure the onboard LED (PA5) as an output.
2. Configure the onboard button (PC13 or another pin) as an input with an interrupt.
3. Use a state machine to track the number of button presses.
4. Use a **millis-like function** to track time and toggle the LED.

### Code Example (Basic OnBoard led blink with millis change via button long press - In Progress) 💻
```c

IN Progress as Testing

```


## Conclusion 🏁
In this lab, you learned how to program the **STM32F103RB-Nucleo board** using **bare metal C** in the **Arduino IDE**. You implemented two use cases:
1. **Delay-based LED flashing speed control**.
2. **Millis-based LED flashing speed control**.

These exercises provide a solid foundation for understanding **register-level programming** and preparing for more advanced topics like **RTOS** and **sensor integration**.

---

## Additional Resources 📚
### STM32F1xx Reference Manual
- [STM32F1xx Reference Manual](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
### ARM Cortex-M3 Technical Reference Manual
- [ARM Cortex-M3 TRM](https://developer.arm.com/documentation/ddi0337/e/)
### STM32duino GitHub Repository
- [STM32duino GitHub](https://github.com/stm32duino)
### Embedded Systems Tutorials
- [DeepBlue Embedded](https://deepbluembedded.com/)
- [Embedded.fm](https://embedded.fm/)
- [Bare-Metal-Tutorial using STMCUbeIDE](https://www.youtube.com/watch?v=UnjaBwNVA_o)

---

