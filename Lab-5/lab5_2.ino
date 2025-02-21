// Transmitter Code (Board with MPU)
#include <Wire.h>
#include <MPU6050.h>
#include <RH_ASK.h>
#include <SPI.h>

// Define pins for RF433
#define RF_TX_PIN 12 // Transmitter data pin
#define RF_RX_PIN 11 // Receiver data pin (if needed for acknowledgment)

// Create RF433 driver instance
RH_ASK rf_driver(2000, RF_RX_PIN, RF_TX_PIN); // Speed = 2000 bits per second

// Create MPU instance
MPU6050 mpu;

// Structure to hold sensor data
struct SensorData
{
    int16_t ax, ay, az; // Accelerometer readings
    int16_t gx, gy, gz; // Gyroscope readings
    float temp;         // Temperature
} data;

// Buffer for data transmission
uint8_t dataBuffer[32]; // Adjust size based on your needs

void setup()
{
    Serial.begin(9600);

    // Initialize RF433
    if (!rf_driver.init())
    {
        Serial.println("RF433 init failed");
        while (1)
            ;
    }

    // Initialize I2C
    Wire.begin();

    // Initialize MPU
    mpu.initialize();

    // Verify MPU connection
    if (!mpu.testConnection())
    {
        Serial.println("MPU connection failed");
        while (1)
            ;
    }

    // Configure MPU settings
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250 °/s
}

void loop()
{
    // Read sensor data
    readSensorData();

    // Pack data into buffer
    packData();

    // Transmit data
    transmitData();

    // Optional: Wait for acknowledgment
    checkAcknowledgment();

    // Delay before next transmission
    delay(100); // Adjust based on your needs
}

void readSensorData()
{
    // Read accelerometer data
    mpu.getAcceleration(&data.ax, &data.ay, &data.az);

    // Read gyroscope data
    mpu.getRotation(&data.gx, &data.gy, &data.gz);

    // Read temperature
    data.temp = mpu.getTemperature() / 340.0 + 36.53; // Convert to celsius
}

void packData()
{
    // Create packet structure
    // Format: [Preamble(2)][Address(1)][Length(1)][Data(n)][Checksum(1)]

    dataBuffer[0] = 0xAA;               // Preamble byte 1
    dataBuffer[1] = 0x55;               // Preamble byte 2
    dataBuffer[2] = 0x01;               // Address (modify for multiple devices)
    dataBuffer[3] = sizeof(SensorData); // Data length

    // Copy sensor data
    memcpy(&dataBuffer[4], &data, sizeof(SensorData));

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 2; i < sizeof(SensorData) + 4; i++)
    {
        checksum ^= dataBuffer[i];
    }
    dataBuffer[sizeof(SensorData) + 4] = checksum;
}

void transmitData()
{
    // Total packet size = Preamble(2) + Address(1) + Length(1) + Data(n) + Checksum(1)
    uint8_t totalSize = sizeof(SensorData) + 5;

    rf_driver.send(dataBuffer, totalSize);
    rf_driver.waitPacketSent();

    // Debug output
    Serial.println("Data transmitted:");
    Serial.print("Acc X: ");
    Serial.println(data.ax);
    Serial.print("Acc Y: ");
    Serial.println(data.ay);
    Serial.print("Acc Z: ");
    Serial.println(data.az);
    Serial.print("Gyro X: ");
    Serial.println(data.gx);
    Serial.print("Gyro Y: ");
    Serial.println(data.gy);
    Serial.print("Gyro Z: ");
    Serial.println(data.gz);
    Serial.print("Temp: ");
    Serial.println(data.temp);
}

void checkAcknowledgment()
{
    // Buffer for acknowledgment
    uint8_t buf[5];
    uint8_t bufLen = sizeof(buf);

    // Wait for a short time for acknowledgment
    unsigned long startTime = millis();
    while (millis() - startTime < 100)
    { // 100ms timeout
        if (rf_driver.recv(buf, &bufLen))
        {
            // Check if it's an acknowledgment packet
            if (buf[0] == 0xAA && buf[1] == 0x55 && buf[2] == 0x01 && buf[3] == 0xAC)
            {
                Serial.println("Acknowledgment received");
                return;
            }
        }
    }
    Serial.println("No acknowledgment received");
}

// Receiver Code (Second Board)
#ifdef RECEIVER_BOARD

void setup()
{
    Serial.begin(9600);

    // Initialize RF433
    if (!rf_driver.init())
    {
        Serial.println("RF433 init failed");
        while (1)
            ;
    }
}

void loop()
{
    uint8_t buf[32]; // Match transmitter buffer size
    uint8_t bufLen = sizeof(buf);

    // Check for received data
    if (rf_driver.recv(buf, &bufLen))
    {
        // Verify preamble
        if (buf[0] == 0xAA && buf[1] == 0x55)
        {
            // Verify address
            if (buf[2] == 0x01)
            { // Match with transmitter address
                // Extract data length
                uint8_t dataLen = buf[3];

                // Verify checksum
                uint8_t checksum = 0;
                for (int i = 2; i < dataLen + 4; i++)
                {
                    checksum ^= buf[i];
                }

                if (checksum == buf[dataLen + 4])
                {
                    // Data is valid, extract sensor readings
                    SensorData receivedData;
                    memcpy(&receivedData, &buf[4], sizeof(SensorData));

                    // Process and display data
                    Serial.println("Received sensor data:");
                    Serial.print("Acc X: ");
                    Serial.println(receivedData.ax);
                    Serial.print("Acc Y: ");
                    Serial.println(receivedData.ay);
                    Serial.print("Acc Z: ");
                    Serial.println(receivedData.az);
                    Serial.print("Gyro X: ");
                    Serial.println(receivedData.gx);
                    Serial.print("Gyro Y: ");
                    Serial.println(receivedData.gy);
                    Serial.print("Gyro Z: ");
                    Serial.println(receivedData.gz);
                    Serial.print("Temp: ");
                    Serial.println(receivedData.temp);

                    // Send acknowledgment
                    sendAcknowledgment();
                }
                else
                {
                    Serial.println("Checksum error");
                }
            }
        }
    }
}

void sendAcknowledgment()
{
    uint8_t ack[5] = {0xAA, 0x55, 0x01, 0xAC, 0xAC}; // Simple acknowledgment packet
    rf_driver.send(ack, sizeof(ack));
    rf_driver.waitPacketSent();
}

#endif