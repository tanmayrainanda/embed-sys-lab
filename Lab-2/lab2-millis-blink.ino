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
    static uint8_t led_state = 0;
    if (led_state)
    {
        GPIOC->BSRR = (1 << LED_PIN); // Set pin high
        led_state = 0;
    }
    else
    {
        GPIOC->BSRR = (1 << (LED_PIN + 16)); // Set pin low
        led_state = 1;
    }
}

void setupClock(void)
{
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // Configure PLL (8MHz HSE => 72MHz)
    RCC->CFGR |= RCC_CFGR_PLLSRC;   // HSE as PLL source
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // Multiply by 9

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Set PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

int main(void)
{
    setupClock(); // Add this first
    setupLED();
    setupButton();
    setupTimer();

    // Add a simple initial toggle to verify LED works
    toggleLED();

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