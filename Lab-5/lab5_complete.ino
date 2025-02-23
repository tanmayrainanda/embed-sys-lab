#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile
#include <Wire.h>
#include <MPU6050_light.h>

RH_ASK driver;
MPU6050 mpu(Wire);

// State machine variables
enum State {RECEIVING, SENDING};
State currentState = SENDING; // Start with SENDING to test immediately

// Timing variables
unsigned long sendStartTime = 0;
const unsigned long SEND_DURATION = 5000; // 5 seconds in milliseconds

void setup()
{
  Serial.begin(9600); // Debugging only
  Wire.begin();
  
  // Initialize radio driver
  if (!driver.init())
    Serial.println("Radio init failed");
    
  // Initialize MPU6050
  byte status = mpu.begin();
  if(status != 0) {
    Serial.println("MPU6050 init failed");
    while(1);
  }
  
  Serial.println("MPU6050 initialized successfully");
  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(); // Calibrate gyro and accelerometer
  Serial.println("Offsets calculated");
  
  // Set starting time immediately
  sendStartTime = millis();
}

void loop()
{
  uint8_t buf[20]; // Buffer for receiving
  uint8_t buflen = sizeof(buf);
  char msgBuffer[40]; // Buffer for sending angle message
  
  // Update MPU data
  mpu.update();

  // Get current angle from MPU6050
  int angleX = (int)(mpu.getAngleX() * 10);
  int angleY = (int)(mpu.getAngleY() * 10);

  // Format the message with tab delimiter and '0' prefix
  sprintf(msgBuffer, "0\t%d,%d\t", angleX, angleY);
  
  switch(currentState) {
    case RECEIVING:
      // Wait until something is received
      if (driver.recv(buf, &buflen)) {
        // Null-terminate the received data
        buf[buflen] = '\0';
        
        // Check if message starts with '0'
        if (buf[0] == '0') {
                // Valid message with prefix '0', print it
                Serial.print("Valid message: ");
                buf[0] = ' ';
                Serial.println((char*)buf);
                // Switch to sending state and record start time
                currentState = SENDING;
                sendStartTime = millis();
                Serial.println("Starting to send angle for 5 seconds...");
                Serial.print("Message: ");
                Serial.println(msgBuffer);
        }
      }
      break;
      
    case SENDING:
      // Check if we've been sending for 5 seconds
      if (millis() - sendStartTime < SEND_DURATION) {
        // Send the angle data
        driver.send((uint8_t *)msgBuffer, strlen(msgBuffer));
        driver.waitPacketSent();
        
        // Small delay to prevent flooding
        delay(200);
      } else {
        // 5 seconds elapsed, switch back to receiving
        currentState = RECEIVING;
        Serial.println("Finished sending, now waiting to receive...");
      }
      break;
  }
}
