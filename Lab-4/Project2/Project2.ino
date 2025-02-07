#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Pin Definitions
const int trigPin = 9;
const int echoPin = 10;

// Constants
const float OBSTACLE_THRESHOLD = 30.0;  // cm
const unsigned long LOG_INTERVAL = 100;  // Log every 100ms

// Sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor
  
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("Failed to find BNO055 chip");
    while (1) {
      delay(10);
    }
  }
  
  delay(1000);
  
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Print calibration status
  displayCalStatus();
  
  // Print CSV header
  Serial.println("Time(ms),Distance(cm),Heading,Roll,Pitch,Obstacle_Detected,Sys_Cal,Gyro_Cal,Accel_Cal,Mag_Cal");
  
  delay(100);
}

void loop() {
  // Read sensor data
  float distance = getFilteredDistance();
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Get calibration status
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  // Log data at specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    bool obstacleDetected = (distance < OBSTACLE_THRESHOLD);
    
    // Log all data
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(event.orientation.x); // Heading
    Serial.print(",");
    Serial.print(event.orientation.z); // Roll
    Serial.print(",");
    Serial.print(event.orientation.y); // Pitch
    Serial.print(",");
    Serial.print(obstacleDetected ? "1" : "0");
    Serial.print(",");
    Serial.print(system);
    Serial.print(",");
    Serial.print(gyro);
    Serial.print(",");
    Serial.print(accel);
    Serial.print(",");
    Serial.println(mag);
    
    lastLogTime = currentTime;
  }
  
  delay(10);  // Small delay for stability
}

float getFilteredDistance() {
  static float lastDistance = 0;
  float currentDistance;
  
  // Get raw distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  currentDistance = (duration * 0.0343) / 2;
  
  // Simple low-pass filter
  currentDistance = (0.7 * currentDistance) + (0.3 * lastDistance);
  lastDistance = currentDistance;
  
  return currentDistance;
}

void displayCalStatus(void) {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  Serial.println("\nCalibration Status:");
  Serial.print("System: ");
  Serial.println(system);
  Serial.print("Gyro: ");
  Serial.println(gyro);
  Serial.print("Accel: ");
  Serial.println(accel);
  Serial.print("Mag: ");
  Serial.println(mag);
}