// ======= Board Configuration =======
// #if defined(ARDUINO_ARCH_ESP32)
// #define BOARD_TYPE "ESP32"
// #define LED_PIN 2
// #define DHT_PIN 4
// #define TRIGGER_PIN 5
// #define ECHO_PIN 18
// Potentiometer pins for ESP32
// const int POT_PINS[] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14};
// #elif defined(ARDUINO_ARCH_STM32)
#define BOARD_TYPE "STM32"
#define LED_PIN PC13
#define DHT_PIN PA1
#define TRIGGER_PIN PA2
#define ECHO_PIN PA3
// Potentiometer pins for STM32
const int POT_PINS[] = {PA0};
// #else // Arduino
// #define BOARD_TYPE "Arduino"
// #define LED_PIN 13
// #define DHT_PIN 2
// #define TRIGGER_PIN 3
// #define ECHO_PIN 4
// Potentiometer pins for Arduino
// const int POT_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
// const int POT_PINS[] = {A0};
// #endif

// ======= Required Libraries =======
#include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>

// ======= Constants & Global Variables =======
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
MPU6050 mpu;

const int NUM_POTS = 1;
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 1000; // Print every second

// ======= 1. Basic LED Blink =======
void setup_led()
{
    pinMode(LED_PIN, OUTPUT);
}

void loop_led()
{
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}

// ======= 2. DHT11 Temperature & Humidity =======
void setup_dht()
{
    dht.begin();
}

void loop_dht()
{
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
}

// ======= 3. HC-SR04 Ultrasonic Distance =======
void setup_ultrasonic()
{
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float measure_distance()
{
    // Clear trigger pin
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);

    // Send 10μs pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Measure the response
    long duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate distance in cm
    return duration * 0.034 / 2;
}

void loop_ultrasonic()
{
    float distance = measure_distance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
}

// ======= 4. GY-521 (MPU6050) IMU =======
void setup_imu()
{
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        return;
    }
}

void loop_imu()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw values to g's and deg/s
    float accel_x = ax / 16384.0;
    float accel_y = ay / 16384.0;
    float accel_z = az / 16384.0;
    float gyro_x = gx / 131.0;
    float gyro_y = gy / 131.0;
    float gyro_z = gz / 131.0;

    Serial.print("Accel (g): ");
    Serial.print(accel_x);
    Serial.print(", ");
    Serial.print(accel_y);
    Serial.print(", ");
    Serial.println(accel_z);

    Serial.print("Gyro (deg/s): ");
    Serial.print(gyro_x);
    Serial.print(", ");
    Serial.print(gyro_y);
    Serial.print(", ");
    Serial.println(gyro_z);
}

// ======= 5. Multiple Potentiometers =======
void setup_pots()
{
    for (int i = 0; i < NUM_POTS; i++)
    {
        pinMode(POT_PINS[i], INPUT);
    }
}

void loop_pots()
{
    if (millis() - lastPrintTime >= PRINT_INTERVAL)
    {
        for (int i = 0; i < NUM_POTS; i++)
        {
            int value = analogRead(POT_PINS[i]);
            Serial.print("Pot ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(value);
            Serial.print(" | ");
        }
        Serial.println();
        lastPrintTime = millis();
    }
}

// ======= Main Program =======
void setup()
{
    Serial.begin(115200);
    Serial.print("Initializing ");
    Serial.print(BOARD_TYPE);
    Serial.println(" board...");

    setup_led();
    setup_dht();
    setup_ultrasonic();
    setup_imu();
    setup_pots();

    Serial.println("Setup complete!");
}

void loop()
{
    // Uncomment the experiment you want to run
    loop_led();
//     loop_dht();
//     loop_ultrasonic();
//     loop_imu();
//     loop_pots();
//     benchmark();

    delay(100); // Small delay to prevent serial buffer overflow
}

// ======= Performance Benchmarking =======
void benchmark()
{
    unsigned long startTime;

    // Memory usage is printed during compilation
    Serial.println("\n=== Performance Benchmark ===");

    // Test DHT11 response time
    startTime = micros();
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    unsigned long dhtTime = micros() - startTime;

    // Test Ultrasonic response time
    startTime = micros();
    float dist = measure_distance();
    unsigned long ultraTime = micros() - startTime;

    // Test IMU response time
    startTime = micros();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long imuTime = micros() - startTime;

    // Test Potentiometer reading time (all channels)
    startTime = micros();
    for (int i = 0; i < NUM_POTS; i++)
    {
        analogRead(POT_PINS[i]);
    }
    unsigned long potTime = micros() - startTime;

    // Print results
    Serial.println("Response Times (microseconds):");
    Serial.print("DHT11: ");
    Serial.println(dhtTime);
    Serial.print("Ultrasonic: ");
    Serial.println(ultraTime);
    Serial.print("IMU: ");
    Serial.println(imuTime);
    Serial.print("10 Pots: ");
    Serial.println(potTime);
}