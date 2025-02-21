# 📡 Lab 5: Peer-to-Peer Communication with RF433

## 🌐 Overview
This lab uses RF433 modules for custom protocol design to implement peer-to-peer communication. You will learn to transmit data between controllers efficiently and explore different communication methods and techniques.

## 🎯 Objectives
- Understand and implement custom communication protocols using RF433 modules.
- Develop a reliable and efficient peer-to-peer communication system.
- Explore techniques to differentiate and maintain peer-to-peer communication among multiple RF433 modules.
- Optimize power consumption and range for low-cost telemetry applications.

## 🏆 Key Activities & Challenges
- **Custom Protocol Design**: Design a custom communication protocol for RF433 modules to ensure reliable data transmission.
- **Bidirectional Communication**: Establish bidirectional communication using RF433 modules.
- **Peer-to-Peer Differentiation**: Develop techniques to differentiate and maintain communication among multiple RF433 modules.
- **Power Optimization**: Implement power-saving techniques to minimize energy consumption.

## 🛠️ Required Components
- **STM32 Nucleo Board**: The primary microcontroller board for this lab.
- **RF433 Modules**: For custom wireless communication.
  - **Features**:
    - Range: Up to 100 meters in open space.
    - Frequency: 433 MHz.
    - Low power consumption.
    - Operating Voltage: 5V for RX, 3V to 6V for TX.
    - Output Power: 4 to 12 dBm for TX.
- **Additional Components**: Jumper wires, breadboard, and power supply.

## 🔧 Implementation Steps

### RF433 Communication

#### Custom Protocol Design
1. **Setup**: Connect the RF433 modules to the STM32 boards.
2. **Code**: Design a custom protocol to handle data transmission, including packet structure, error checking, and addressing.
   - **Packet Structure**:
     - Preamble: Fixed sequence to signal the start of a packet.
     - Address: Unique identifier for each node.
     - Data: Payload containing telemetry data.
     - Error Checking: CRC or checksum for error detection.
     - Encryption: Optional lightweight encryption for secure communication.
3. **Test**: Transmit and receive data between the boards using the custom protocol.

#### Bidirectional Communication
1. **Setup**: Ensure both STM32 boards are configured for bidirectional communication using RF433 modules.
2. **Code**: Implement the code to handle both sending and receiving data.
3. **Test**: Exchange data bidirectionally between the boards.

#### Peer-to-Peer Differentiation
1. **Addressing**: Assign unique addresses to each RF433 module to differentiate them.
2. **Code**: Implement address filtering in the communication protocol to ensure data is sent to the intended recipient.
3. **Test**: Verify that data is correctly routed to the intended module.

### Power Management and Optimization
1. **Duty Cycling**: Implement duty cycling to reduce power consumption by turning off the RF433 modules when not in use.
2. **Adaptive Power Control**: Adjust the transmission power based on the distance between nodes to save energy.
3. **Sleep Modes**: Utilize sleep modes to minimize power consumption during idle periods.

## 📚 Resources
- **STM32 Documentation**: [Official STM32 Documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
- **RF433 Module Datasheet**: [RF433 Module Datasheet](https://cdn.sparkfun.com/datasheets/Wireless/General/RWS-371-6_433.92MHz_ASK_RF_Receiver_Module_Data_Sheet.pdf)

## 🤝 Contribution Guidelines
- Fork the repository and create a new branch for your contributions.
- Ensure your code is well-documented and follows the existing structure.
- Submit a pull request with a detailed description of your changes.

---

### Sample Code for Bidirectional Communication

#### Board 1 (Transmitter and Receiver)

```cpp
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
```

#### Board 2 (Transmitter and Receiver)

```cpp
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
```

### Explanation

- **Initialization**: The `RH_ASK` driver is initialized for RF communication.
- **Receiving Messages**: Each board listens for incoming messages. If a message is received, it checks if the message is "Hi".
- **Sending Messages**: If "Hi" is received, the board responds with "Hello". Each board also periodically sends "Hi" to initiate communication.
- **Bidirectional Communication**: Both boards act as transmitters and receivers, ensuring bidirectional communication.

### Additional Considerations

- **Addressing**: For a more complex network, implement addressing to differentiate between multiple nodes.
- **Error Handling**: Add error handling for robust communication, such as retransmission on failure.
- **Power Management**: Implement power-saving techniques like sleep modes and duty cycling to minimize power consumption.

### Relevant Reources

- **Prototype**: [RF-433](https://lastminuteengineers.com/433mhz-rf-wireless-arduino-tutorial/)
- **Working**: [RF-433](https://www.wellpcb.com/blog/pcb-basics/433mhz/#:~:text=Generally%2C%20mesh%20networking%20permits%20devices%20to%20relay%20signals,Sadly%2C%20433MHz%20devices%20can%E2%80%99t%20construct%20a%20mesh%20network.)
- **Test Build**: [RF-433](https://how2electronics.com/433mhz-rf-module-works-interfacing-arduino/)


#### This setup ensures that each board can communicate bidirectionally using RF433 modules, with a simple "Hi" and "Hello" message exchange to verify the communication link.
