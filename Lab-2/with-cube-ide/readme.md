# Bare-Metal with STM32CubeIDE on STM32F103RBT6

## Onboard LED Blinking Example

This guide demonstrates how to toggle the onboard LED of the STM32F103RBT6 microcontroller using GPIO pin `PA5`. 
The example is built using STM32CubeIDE in a bare-metal environment.

---

## 📋 Prerequisites

Before starting, ensure you have the following:

### Hardware Requirements
- STM32F103RBT6 Microcontroller Board
- ST-Link Debugger
- USB Cable (for power and programming)

### Software Tools
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

# Setup Environment in STM32CubeIDE

To get started with your STM32 development using STM32CubeIDE, you need to follow a few steps to set up your environment. This includes installing STM32CubeIDE, creating a new project, configuring the necessary GPIO pins, and programming the LED to blink. 

For an easy walkthrough of these setup steps, refer to this helpful article:

Article: [STM32 Nucleo GPIO Pins and LED Blinking in STM32CubeIDE](https://microcontrollerslab.com/stm32-nucleo-gpio-pins-led-blinking-stm32cubeide/)

This article provides step-by-step instructions to:

- Install STM32CubeIDE
- Create and configure a new STM32 project
- Set up the GPIO pins for LED control
- Implement basic LED blinking logic

Follow the article's instructions to get your STM32 project up and running before proceeding to the rest of the steps in this README.

---

## 🛠️ Project Setup

### Step 1: Create a New Project
1. Open **STM32CubeIDE**.
2. Create a new project by selecting the **STM32F103RBT6** microcontroller.
3. Set up a blank project, allowing STM32CubeIDE to generate the necessary HAL drivers.

### Step 2: GPIO Configuration
1. Open the **Pinout & Configuration** tab.
2. Configure `PA5` (labeled as LED on the board) as a GPIO output pin:
   - Pin Mode: `Output Push Pull`
   - Pull: `No Pull`
   - Speed: `Low`
     
   ![image](https://github.com/user-attachments/assets/ac218cea-b747-42c3-b3da-b882a176006b)

- locating "main.c"
  
  ![image](https://github.com/user-attachments/assets/bcaf3bb1-efc0-44d2-8a82-250012df4d67)

---

## 📄 Code Explanation

Below is the example code to toggle the onboard LED connected to pin `PA5`:

### `main.c`
```c
/* USER CODE BEGIN Header */
/**
  ****************************************************************************** 
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************** 
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h" // Include the header file for main, which contains necessary definitions

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_5 // Define the GPIO pin connected to the LED (PA5)
#define LED_GPIO_PORT GPIOA // Define the GPIO port (GPIOA) where the LED is connected
#define DELAY_MS 1000 // Define a delay of 1000 ms (1 second)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // Prototype for the function to configure the system clock
static void MX_GPIO_Init(void); // Prototype for the function to initialize GPIO

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Initialize the Hardware Abstraction Layer (HAL), setting up system and peripherals

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Configure the system clock (using HSI oscillator)

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Initialize GPIO for LED control

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // Toggle the LED (turn it on/off)
    HAL_Delay(DELAY_MS); // Introduce a delay of 1000 ms (1 second)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Initialize a struct to configure the oscillator settings
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Initialize a struct to configure the clock settings

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Use the internal HSI (High-Speed Internal) oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Turn on the HSI oscillator
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Use the default calibration value for HSI
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // Disable the Phase-Locked Loop (PLL)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) // Apply the oscillator settings
  {
    Error_Handler(); // If configuration fails, call error handler
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Select the clocks to configure
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // Set HSI as the system clock source
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // Set AHB clock divider to 1 (no division)
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // Set APB1 clock divider to 1 (no division)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // Set APB2 clock divider to 1 (no division)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) // Apply the clock settings
  {
    Error_Handler(); // If configuration fails, call error handler
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // Initialize a struct for GPIO configuration

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA (where the LED is connected)

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET); // Set the LED pin to low (LED OFF)

  /*Configure GPIO pin : LED_PIN */
  GPIO_InitStruct.Pin = LED_PIN; // Configure the pin for LED (PA5)
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set the pin as a push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistors
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set output speed to low
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct); // Apply the GPIO configuration to the LED pin
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq(); // Disable interrupts in case of error
  while (1)
  {
    // Infinite loop to keep the MCU in error state
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // This function will be executed when an assert fails
}
#endif /* USE_FULL_ASSERT */


```

## ⚙️ How It Works

### Initialization
- **System Clock**: The system clock is configured using the `SystemClock_Config` function.
- **GPIO Initialization**: GPIO initialization is performed in the `MX_GPIO_Init` function.

### Main Loop
- **LED Toggle**: The LED is toggled on/off with `HAL_GPIO_TogglePin`.
- **Delay**: A delay of 1 second is introduced using `HAL_Delay`.

## 🚀 Building and Flashing

### Build the Project
- Click the **Build** button in STM32CubeIDE to compile the project.

 ![image](https://github.com/user-attachments/assets/d4df5ea6-ee63-415f-9629-1b78bccef947)

### Flash to MCU
- Connect the STM32 board via **ST-Link**.
- Click the **Debug** or **Run** button in STM32CubeIDE to upload the code.

 ![image](https://github.com/user-attachments/assets/c530f2b7-3a92-4c7f-9117-290ad02817fc)

## 🧪 Experiment
- Modify the `DELAY_MS` value to change the LED's blinking speed.

## 📝 Notes
- Ensure the clock settings are correct for stable GPIO operation.
- Pin **PA5** is pre-configured as the onboard LED, eliminating the need for external components.

---

# 📝 Notes
- Ensure the clock settings are correct for stable GPIO operation.
- Pin `PA5` is pre-configured as the onboard LED, eliminating the need for external components.
