/* Includes */
#include "main.h"
#include "dht11.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

/* Define thresholds */
#define TEMP_MAX_THRESHOLD 30.0f
#define DISTANCE_MIN_THRESHOLD 10.0f // cm
#define TILT_THRESHOLD 45.0f         // degrees

/* Pin definitions */
#define DHT11_PIN GPIO_PIN_0
#define DHT11_PORT GPIOA
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_1
#define ECHO_PORT GPIOB

/* Global variables */
float temperature = 0.0f;
float humidity = 0.0f;
float distance = 0.0f;
float tilt_angle = 0.0f;

/* Function prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void I2C1_Init(void);
float measure_distance(void);
void check_thresholds(void);
void MPU6050_Process(void);

int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    I2C1_Init();

    /* Initialize MPU6050 */
    MPU6050_Init();

    /* Initialize UART for serial monitor */
    MX_USART2_UART_Init();

    while (1)
    {
        /* Read DHT11 sensor */
        if (DHT11_Read(&temperature, &humidity) == HAL_OK)
        {
            printf("Temperature: %.1f°C, Humidity: %.1f%%\r\n",
                   temperature, humidity);
        }

        /* Measure distance */
        distance = measure_distance();
        printf("Distance: %.1f cm\r\n", distance);

        /* Get tilt data from MPU6050 */
        MPU6050_Process();

        /* Check for threshold violations */
        check_thresholds();

        /* Delay before next reading */
        HAL_Delay(2000);
    }
}

float measure_distance(void)
{
    uint32_t pulse_duration;
    float distance_cm;

    /* Generate 10us trigger pulse */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    HAL_Delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* Wait for echo pulse */
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
        ;

    /* Measure pulse duration */
    uint32_t start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
        ;
    pulse_duration = HAL_GetTick() - start_time;

    /* Calculate distance */
    distance_cm = (pulse_duration * 0.034) / 2;

    return distance_cm;
}

void MPU6050_Process(void)
{
    MPU6050_t MPU6050_Data;

    /* Read accelerometer and gyroscope data */
    MPU6050_Read_All(&MPU6050_Data);

    /* Calculate tilt angle using accelerometer data */
    tilt_angle = atan2(MPU6050_Data.Accel_X_RAW,
                       sqrt(pow(MPU6050_Data.Accel_Y_RAW, 2) +
                            pow(MPU6050_Data.Accel_Z_RAW, 2))) *
                 180 / M_PI;

    printf("Tilt Angle: %.1f degrees\r\n", tilt_angle);
}

void check_thresholds(void)
{
    /* Temperature threshold check */
    if (temperature > TEMP_MAX_THRESHOLD)
    {
        printf("ALERT: High temperature detected!\r\n");
    }

    /* Distance threshold check */
    if (distance < DISTANCE_MIN_THRESHOLD)
    {
        printf("ALERT: Object too close!\r\n");
    }

    /* Tilt threshold check */
    if (abs(tilt_angle) > TILT_THRESHOLD)
    {
        printf("ALERT: Excessive tilt detected!\r\n");
    }
}

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Ports Clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure DHT11 pin */
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    /* Configure Ultrasonic sensor pins */
    GPIO_InitStruct.Pin = TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}

void I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c1);
}