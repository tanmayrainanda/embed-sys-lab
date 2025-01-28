
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
