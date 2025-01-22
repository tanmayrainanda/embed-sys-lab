# 🌟 Lab 2: Bare Metal Programming on STM32F103RB-Nucleo

Welcome to the **Bare Metal Programming Lab**! In this lab, you will learn how to program the **STM32F103RB-Nucleo board** using **bare metal C** in the **Arduino IDE**. You will implement two use cases:
1. **LED Flashing Speed Control Using Delay** ⏳
2. **LED Flashing Speed Control Using Millis** ⏱️

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
  - Arduino IDE with STM32duino support (installed via STM32 Core).
  - STM32F1xx HAL library (optional for reference).
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
1. Configure the onboard LED (PC13) as an output.
2. Configure the onboard button (PC13 or another pin) as an input with an interrupt.
3. Use a state machine to track the number of button presses.
4. Change the LED flashing speed based on the state.

### Code Example 💻
```c
#include "stm32f1xx.h"

volatile uint8_t button_presses = 0;
volatile uint32_t delay_time = 500; // Default delay

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;  // Clear pending bit
        button_presses++;
        if (button_presses > 3) button_presses = 1;

        // Update delay time based on button presses
        if (button_presses == 1) delay_time = 50;
        else if (button_presses == 2) delay_time = 500;
        else if (button_presses == 3) delay_time = 1000;
    }
}

void delay(volatile uint32_t count) {
    while (count--);
}

int main(void) {
    // Enable GPIOC and AFIO clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    // Configure PC13 as output (onboard LED)
    GPIOC->CRH &= ~(0xF << 20);  // Clear mode for PC13
    GPIOC->CRH |= (0x3 << 20);   // Set PC13 as output

    // Configure PA0 as input (onboard button)
    GPIOA->CRL &= ~(0xF << 0);   // Clear mode for PA0
    GPIOA->CRL |= (0x4 << 0);    // Set PA0 as input

    // Configure EXTI0 for PA0
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;  // Select PA0 for EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;   // Enable EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; // Enable rising edge trigger

    // Enable EXTI0 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    while (1) {
        GPIOC->ODR ^= (1 << 13);  // Toggle PC13 (onboard LED)
        delay(delay_time * 1000); // Delay based on button presses
    }
}
```
## Use Case 2: LED Flashing Speed Control Using Millis ⏱️

### Objective 🎯
Control the flashing speed of the onboard LED using the onboard button. Instead of using a delay function, use a **millis-based approach** to track time and toggle the LED.

### Steps 🛠️
1. Configure the onboard LED (PC13) as an output.
2. Configure the onboard button (PC13 or another pin) as an input with an interrupt.
3. Use a state machine to track the number of button presses.
4. Use a **millis-like function** to track time and toggle the LED.

### Code Example 💻
```c
#include "stm32f1xx.h"

volatile uint8_t button_presses = 0;
volatile uint32_t interval = 500; // Default interval
uint32_t previous_millis = 0;

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;  // Clear pending bit
        button_presses++;
        if (button_presses > 3) button_presses = 1;

        // Update interval based on button presses
        if (button_presses == 1) interval = 50;
        else if (button_presses == 2) interval = 500;
        else if (button_presses == 3) interval = 1000;
    }
}

uint32_t millis(void) {
    return SysTick->VAL; // Use SysTick timer for millis-like functionality
}

int main(void) {
    // Enable GPIOC and AFIO clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    // Configure PC13 as output (onboard LED)
    GPIOC->CRH &= ~(0xF << 20);  // Clear mode for PC13
    GPIOC->CRH |= (0x3 << 20);   // Set PC13 as output

    // Configure PA0 as input (onboard button)
    GPIOA->CRL &= ~(0xF << 0);   // Clear mode for PA0
    GPIOA->CRL |= (0x4 << 0);    // Set PA0 as input

    // Configure EXTI0 for PA0
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;  // Select PA0 for EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;   // Enable EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; // Enable rising edge trigger

    // Enable EXTI0 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Configure SysTick timer for millis-like functionality
    SysTick->LOAD = 72000 - 1;   // 1 ms interval (72 MHz / 1000)
    SysTick->VAL = 0;            // Clear current value
    SysTick->CTRL = 0x07;        // Enable SysTick, use processor clock

    while (1) {
        uint32_t current_millis = millis();
        if (current_millis - previous_millis >= interval) {
            previous_millis = current_millis;
            GPIOC->ODR ^= (1 << 13);  // Toggle PC13 (onboard LED)
        }
    }
}
```
## Conclusion 🏁
In this lab, you learned how to program the **STM32F103RB-Nucleo board** using **bare metal C** in the **Arduino IDE**. You implemented two use cases:
1. **Delay-based LED flashing speed control**.
2. **Millis-based LED flashing speed control**.

These exercises provide a solid foundation for understanding **register-level programming** and preparing for more advanced topics like **RTOS** and **sensor integration**.

---

## Additional Resources 📚
- [STM32F103RB Reference Manual](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [STM32duino GitHub Repository](https://github.com/stm32duino)
- [ARM Cortex-M3 Technical Reference Manual](https://developer.arm.com/documentation/ddi0337/e/)
- [Bare-Metal-Tutorial using STMCUbeIDE](https://www.youtube.com/watch?v=UnjaBwNVA_o)

---

