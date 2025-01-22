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
