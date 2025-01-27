#include "stm32f1xx.h"

volatile uint8_t button_presses = 0;
volatile uint32_t interval = 500; // Default interval
uint32_t previous_millis = 0;

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR |= EXTI_PR_PR0; // Clear pending bit
        button_presses++;
        if (button_presses > 3)
            button_presses = 1;

        // Update interval based on button presses
        if (button_presses == 1)
            interval = 50;
        else if (button_presses == 2)
            interval = 500;
        else if (button_presses == 3)
            interval = 1000;
    }
}

uint32_t millis(void)
{
    return SysTick->VAL; // Use SysTick timer for millis-like functionality
}

int main(void)
{
    // Enable GPIOC and AFIO clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    // Configure PC13 as output (onboard LED)
    GPIOC->CRH &= ~(0xF << 20); // Clear mode for PC13
    GPIOC->CRH |= (0x3 << 20);  // Set PC13 as output

    // Configure PA0 as input (onboard button)
    GPIOA->CRL &= ~(0xF << 0); // Clear mode for PA0
    GPIOA->CRL |= (0x4 << 0);  // Set PA0 as input

    // Configure EXTI0 for PA0
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Select PA0 for EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;                // Enable EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0;              // Enable rising edge trigger

    // Enable EXTI0 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Configure SysTick timer for millis-like functionality
    SysTick->LOAD = 72000 - 1; // 1 ms interval (72 MHz / 1000)
    SysTick->VAL = 0;          // Clear current value
    SysTick->CTRL = 0x07;      // Enable SysTick, use processor clock

    while (1)
    {
        uint32_t current_millis = millis();
        if (current_millis - previous_millis >= interval)
        {
            previous_millis = current_millis;
            GPIOC->ODR ^= (1 << 13); // Toggle PC13 (onboard LED)
        }
    }
}