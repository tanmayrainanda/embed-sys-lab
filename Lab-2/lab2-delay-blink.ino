#include "stm32f1xx.h"

// Constants
#define LED_PIN 13
#define BUTTON_PIN 0
#define BLINK_FAST 50
#define BLINK_MEDIUM 500
#define BLINK_SLOW 1000

// Global variables
volatile uint8_t button_presses = 0;
volatile uint32_t interval = BLINK_MEDIUM;
volatile uint32_t system_ticks = 0;
uint32_t previous_millis = 0;

// Function prototypes
void setupLED(void);
void setupButton(void);
void setupTimer(void);
void toggleLED(void);
uint32_t millis(void);

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR |= EXTI_PR_PR0; // Clear pending bit
        button_presses++;
        if (button_presses > 3)
            button_presses = 1;

        switch (button_presses)
        {
        case 1:
            interval = BLINK_FAST;
            break;
        case 2:
            interval = BLINK_MEDIUM;
            break;
        case 3:
            interval = BLINK_SLOW;
            break;
        }
    }
}

void SysTick_Handler(void)
{
    system_ticks++;
}

uint32_t millis(void)
{
    return system_ticks;
}

void setupLED(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xF << (4 * (LED_PIN - 8)));
    GPIOC->CRH |= (0x3 << (4 * (LED_PIN - 8)));
}

void setupButton(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRL &= ~(0xF << (4 * BUTTON_PIN));
    GPIOA->CRL |= (0x4 << (4 * BUTTON_PIN));

    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void setupTimer(void)
{
    SysTick->LOAD = 72000 - 1; // 1ms interval at 72MHz
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

void toggleLED(void)
{
    GPIOC->ODR ^= (1 << LED_PIN);
}

int main(void)
{
    setupLED();
    setupButton();
    setupTimer();

    while (1)
    {
        uint32_t current_millis = millis();
        if (current_millis - previous_millis >= interval)
        {
            previous_millis = current_millis;
            toggleLED();
        }
    }
}