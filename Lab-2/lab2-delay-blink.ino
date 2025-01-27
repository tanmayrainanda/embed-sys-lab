#include "stm32f1xx.h"

// Constants
#define LED_PIN 13
#define BUTTON_PIN 0
#define DELAY_FAST 50
#define DELAY_MEDIUM 500
#define DELAY_SLOW 1000

typedef enum
{
    LED_OFF = 0,
    LED_ON = 1
} LedState;

// Global variables
volatile uint8_t button_presses = 0;
volatile uint32_t delay_time = DELAY_MEDIUM;
volatile uint32_t ticks = 0;

// Function prototypes
static void initSystem(void);
static void initLED(void);
static void initButton(void);
static void initSysTick(void);
static void setLED(LedState state);
static void toggleLED(void);

void SysTick_Handler(void)
{
    ticks++;
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR |= EXTI_PR_PR0;
        button_presses++;
        if (button_presses > 3)
            button_presses = 1;

        switch (button_presses)
        {
        case 1:
            delay_time = DELAY_FAST;
            break;
        case 2:
            delay_time = DELAY_MEDIUM;
            break;
        case 3:
            delay_time = DELAY_SLOW;
            break;
        }
    }
}

static void initSystem(void)
{
    initLED();
    initButton();
    initSysTick();
}

static void initLED(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xF << (4 * (LED_PIN - 8)));
    GPIOC->CRH |= (0x3 << (4 * (LED_PIN - 8)));
}

static void initButton(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRL &= ~(0xF << (4 * BUTTON_PIN));
    GPIOA->CRL |= (0x4 << (4 * BUTTON_PIN));

    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    NVIC_EnableIRQ(EXTI0_IRQn);
}

static void initSysTick(void)
{
    SysTick->LOAD = 72000 - 1; // 1ms @ 72MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 7; // Enable, interrupt enable, use CPU clock
}

static void setLED(LedState state)
{
    if (state == LED_ON)
    {
        GPIOC->BSRR = (1 << LED_PIN);
    }
    else
    {
        GPIOC->BSRR = (1 << (LED_PIN + 16));
    }
}

static void toggleLED(void)
{
    GPIOC->ODR ^= (1 << LED_PIN);
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ticks;
    while ((ticks - start) < ms)
        ;
}

int main(void)
{
    initSystem();

    while (1)
    {
        toggleLED();
        delay_ms(delay_time);
    }
}