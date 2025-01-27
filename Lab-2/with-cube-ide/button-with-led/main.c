/* Includes ------------------------------------------------------------------*/
// Includes the main header file, which contains declarations for peripheral drivers,
// application-specific macros, and configurations.
#include "main.h"

/* Private define ------------------------------------------------------------*/
// Define the pin and port for the LED.
#define LED_PIN GPIO_PIN_5       // GPIO Pin 5 is used for the LED.
#define LED_GPIO_PORT GPIOA      // The LED is connected to GPIO Port A.

// Define the pin and port for the button.
#define BUTTON_PIN GPIO_PIN_13   // GPIO Pin 13 is used for the button.
#define BUTTON_GPIO_PORT GPIOC   // The button is connected to GPIO Port C.

// Define a debounce delay to avoid registering multiple presses due to mechanical bounce.
#define DEBOUNCE_DELAY 50        // 50 ms debounce delay.

/* Private variables ---------------------------------------------------------*/
// Variable to hold the current LED blink delay. The default is 1000 ms (1 second).
uint32_t blink_delay = 1000;

// A flag to track the button's pressed state. Not used in this version but can be for future logic.
uint8_t button_pressed = 0;

/* Private function prototypes -----------------------------------------------*/
// Function prototypes for system configuration and GPIO initialization.
void SystemClock_Config(void);   // Configures the system clock.
static void MX_GPIO_Init(void); // Initializes the GPIO pins.

// Prototype for the callback function to handle button interrupts.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* Main Program --------------------------------------------------------------*/
int main(void)
{
  HAL_Init();                  // Initializes the HAL library and configures the system.
  SystemClock_Config();        // Configures the system clock.
  MX_GPIO_Init();              // Initializes GPIOs for LED and button.

  while (1)                    // Main loop that runs indefinitely.
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // Toggles the LED state (ON/OFF).
    HAL_Delay(blink_delay);    // Waits for the duration specified by `blink_delay`.
  }
}

/* Interrupt Handler for Button Press ----------------------------------------*/
// Callback function triggered when an external interrupt is detected on the button pin.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_interrupt_time = 0; // Stores the time of the last interrupt to handle debounce.
  uint32_t current_time = HAL_GetTick();   // Gets the current system time in milliseconds.

  // Check if the interrupt is for the button pin and if debounce delay has passed.
  if (GPIO_Pin == BUTTON_PIN && (current_time - last_interrupt_time > DEBOUNCE_DELAY))
  {
    // Cycle through different blink delays: 50 ms, 500 ms, 1000 ms.
    if (blink_delay == 50)
      blink_delay = 500;       // Change to 500 ms if the current delay is 50 ms.
    else if (blink_delay == 500)
      blink_delay = 1000;      // Change to 1000 ms if the current delay is 500 ms.
    else
      blink_delay = 50;        // Change to 50 ms if the current delay is 1000 ms.

    last_interrupt_time = current_time; // Update the last interrupt time.
  }
}

/* System Clock Configuration ------------------------------------------------*/
// Configures the system clock settings for the microcontroller.
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Structure for oscillator configuration.
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Structure for clock configuration.

  // Configure the High-Speed Internal (HSI) oscillator.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI oscillator.
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Use default calibration.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // Disable PLL (not used in this configuration).
  HAL_RCC_OscConfig(&RCC_OscInitStruct); // Apply the oscillator configuration.

  // Configure the system clock sources and dividers.
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // Use HSI as the system clock source.
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // No division for AHB clock.
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      // No division for APB1 clock.
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // No division for APB2 clock.
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0); // Apply the clock configuration.
}

/* GPIO Initialization Function ----------------------------------------------*/
// Configures GPIO pins for the LED and button.
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // Structure to configure GPIO settings.

  /* Enable GPIO Ports Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIO Port A (LED).
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable clock for GPIO Port C (Button).

  /* Configure LED GPIO Pin */
  GPIO_InitStruct.Pin = LED_PIN;               // Select the LED pin.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Set as push-pull output.
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // No internal pull-up or pull-down.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low frequency (reduces power consumption).
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct); // Initialize the LED pin.

  /* Configure Button GPIO Pin */
  GPIO_InitStruct.Pin = BUTTON_PIN;             // Select the button pin.
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Set as interrupt on falling edge.
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Enable internal pull-up for the button.
  HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct); // Initialize the button pin.

  /* Enable and Set EXTI Line Interrupt Priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0); // Set EXTI interrupt priority.
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         // Enable EXTI interrupt for lines 10 to 15.
}

/* Error Handler -------------------------------------------------------------*/
// Error handler function. It enters an infinite loop to indicate an error.
void Error_Handler(void)
{
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
// Function triggered on assertion failure (used for debugging).
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
