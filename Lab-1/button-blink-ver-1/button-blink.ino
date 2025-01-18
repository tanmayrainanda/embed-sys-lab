// Define constants
#define BUTTON_PIN USER_BTN // Onboard button (typically USER button on Nucleo)
#define LED_PIN LED_BUILTIN  // Onboard LED

// Variables to track button presses and delay
int pressCount = 0;
const int delayLevels[] = {50, 100, 500}; // Three levels of delay
int currentDelay = delayLevels[0];

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Internal pull-up resistor activated
}

void loop() {
    // Check if button is pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(10); // Debounce delay
        while (digitalRead(BUTTON_PIN) == LOW); // Wait until button is released
        delay(10); // Debounce delay

        // Increment press count and loop through delay levels
        pressCount = (pressCount + 1) % 3;
        currentDelay = delayLevels[pressCount];
    }
    
    // Blink LED
    digitalWrite(LED_PIN, HIGH);
    delay(currentDelay);
    digitalWrite(LED_PIN, LOW);
    delay(currentDelay);
}
