#include <RH_ASK.h>
#include <SPI.h>

// Define the radio driver
RH_ASK rf_driver;

void setup() {
    Serial.begin(9600);
    if (!rf_driver.init())
        Serial.println("Init failed");
}

void loop() {
    // Buffer to store incoming messages
    uint8_t buf[12];
    uint8_t buflen = sizeof(buf);

    // Check if a message is available
    if (rf_driver.recv(buf, &buflen)) {
        buf[buflen] = '\0'; // Null-terminate the string
        Serial.print("Received: ");
        Serial.println((char*)buf);

        // Respond with "Hello" if "Hi" is received
        if (strcmp((char*)buf, "Hi") == 0) {
            const char *msg = "Hello";
            rf_driver.send((uint8_t *)msg, strlen(msg));
            rf_driver.waitPacketSent();
            Serial.println("Sent: Hello");
        }
    }

    // Send "Hi" periodically
    const char *msg = "Hi";
    rf_driver.send((uint8_t *)msg, strlen(msg));
    rf_driver.waitPacketSent();
    Serial.println("Sent: Hi");

    delay(2000); // Wait for 2 seconds before sending the next message
}