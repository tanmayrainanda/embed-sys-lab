# 🌟 Cheat Sheet For STM32F103RB-Nucleo Bare Metal  

This cheat sheet provides a comprehensive guide to **bare metal programming** on the **STM32F103RB-Nucleo board**. It includes syntax, commands, and logic to interact with the system at a **register level**.

---

## **STM32F103RB-Nucleo Board Overview**
- **Microcontroller**: STM32F103RB (ARM Cortex-M3)
- **Clock Speed**: 72 MHz
- **Flash Memory**: 128 KB
- **SRAM**: 20 KB
- **GPIO Pins**: 51 (with alternate functions)
- **Onboard Peripherals**:
  - LED (connected to PC13)
  - Button (connected to PC13 or another GPIO pin)
  - USB connector for programming and power
  - Arduino-compatible headers

---

## **Architecture Overview**
The **STM32F103RB** microcontroller is based on the **ARM Cortex-M3** architecture. It consists of:
1. **CPU Core**: ARM Cortex-M3 with a 3-stage pipeline.
2. **Memory**:
   - Flash memory for program storage.
   - SRAM for data storage.
3. **Peripherals**:
   - GPIO, Timers, USART, SPI, I2C, ADC, etc.
4. **Clock System**:
   - Internal RC oscillator (HSI) or external crystal (HSE).
   - PLL for clock multiplication.
5. **Interrupts**:
   - Nested Vectored Interrupt Controller (NVIC) for handling interrupts.

**Architecture Image**:

![STM32F103RB Architecture](https://github.com/user-attachments/assets/cd0bcd39-ada1-434e-853c-82dea4a7e1b2)  
*(Refer to the official STM32F103RB datasheet for a detailed architecture diagram.)*

---

## **Cheat Sheet: Bare Metal Programming**

### **1. GPIO Configuration**
- **Set Pin as Output**:
  ```c
  GPIOx->CRL |= (0x3 << (pin * 4));  // For pins 0-7
  GPIOx->CRH |= (0x3 << ((pin - 8) * 4));  // For pins 8-15
  ```

- **Set Pin as Input**:
  ```c
  GPIOx->CRL &= ~(0xF << (pin * 4));  // For pins 0-7
  GPIOx->CRH &= ~(0xF << ((pin - 8) * 4));  // For pins 8-15
  Toggle Pin State:
  ```
  ```c
  GPIOx->ODR ^= (1 << pin);
  ```

### **2. Clock Configuration**
- **Enable GPIO Clock**:
  
  ```c
  RCC->APB2ENR |= RCC_APB2ENR_IOPxEN;  // Replace 'x' with the port letter (A, B, C, etc.)
  Enable Alternate Function Clock:
  ```
  ```c
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  ```

### **3. Interrupts**
-  **Enable EXTI Interrupt**:
  
    ```c
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTIx_Px;  // Configure EXTI for pin x
    EXTI->IMR |= EXTI_IMR_MRx;  // Enable EXTI interrupt for line x
    EXTI->RTSR |= EXTI_RTSR_TRx;  // Enable rising edge trigger
    NVIC_EnableIRQ(EXTIx_IRQn);  // Enable interrupt in NVIC
    Interrupt Service Routine (ISR):
    ```
    ```c
      void EXTIx_IRQHandler(void) {
        if (EXTI->PR & EXTI_PR_PRx) {
            EXTI->PR |= EXTI_PR_PRx;  // Clear pending bit
            // Your code here
        }
    }
    ```

### **4. Timers**
- **Enable Timer Clock**:
  
    ```c
    RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;  // Replace 'x' with the timer number (2, 3, etc.)
    Configure Timer:
    ```
    ```c
    TIMx->PSC = 7200 - 1;  // Prescaler for 10 kHz clock
    TIMx->ARR = 1000 - 1;  // Auto-reload value for 1 second interval
    TIMx->CR1 |= TIM_CR1_CEN;  // Enable timer
    ```

### **5. UART Communication**
-  **Enable UART Clock**:
  
    ```c
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    Configure UART:
    ```
    ```c
    USART1->BRR = 0x1D4C;  // Baud rate for 9600 at 72 MHz
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Enable transmitter, receiver, and UART
    ```

### **6. ADC (Analog-to-Digital Conversion)**
-  **Enable ADC Clock**:
  
    ```c
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    Configure ADC:
    ```
    ```c
    ADC1->SQR3 = 0x0;  // Select channel 0
    ADC1->CR2 |= ADC_CR2_ADON;  // Enable ADC
    ```

### **7. SysTick Timer (for Millis)**
-  **Configure SysTick**:
  
    ```c
    SysTick->LOAD = 72000 - 1;  // 1 ms interval (72 MHz / 1000)
    SysTick->VAL = 0;  // Clear current value
    SysTick->CTRL = 0x07;  // Enable SysTick, use processor clock
    Millis Function:
    ```
    ```c
    uint32_t millis(void) {
        return SysTick->VAL;
    }
    ```

## Common Bare Metal Programming Logic
-  **State Machine**:
    Use a switch-case statement to manage different states.
  
    Example:

      ```c
      enum {STATE_OFF, STATE_ON, STATE_BLINKING} state = STATE_OFF;
      switch (state) {
          case STATE_OFF: break;
          case STATE_ON: break;
          case STATE_BLINKING: break;
      }
      ```

## Debouncing:

-  Use a delay or timer to debounce button presses.
    Example:
   
      ```c
      if (button_pressed) {
          delay(50);  // Wait 50 ms
          if (button_pressed) {
              // Valid press
          }
      }
      ```

## Conclusion
This cheat sheet provides a **comprehensive guide** to bare metal programming on the **STM32F103RB-Nucleo board**. It covers GPIO, interrupts, timers, UART, ADC, and more. Use this as a reference to explore the full potential of the STM32F103RB microcontroller.

---
## Additional Resources
1. **STM32F103RB Reference Manual**:
   - [STM32F103RB Reference Manual](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
2. **ARM Cortex-M3 Technical Reference Manual**:
   - [ARM Cortex-M3 TRM](https://developer.arm.com/documentation/ddi0337/e/)
3. **STM32duino GitHub Repository**:
   - [STM32duino GitHub](https://github.com/stm32duino)
4. **Embedded Systems Tutorials**:
   - [DeepBlue Embedded](https://deepbluembedded.com/)
   - [Embedded.fm](https://embedded.fm/)
---

This cheat sheet is designed to help in **Exploring & Maximizing your understanding** of bare metal programming on the STM32F103RB-Nucleo board.
