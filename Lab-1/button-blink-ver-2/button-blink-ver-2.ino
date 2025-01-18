// Define constants
#define BUTTON_PIN USER_BTN  // Onboard button (typically USER button on Nucleo)
#define LED_PIN LED_BUILTIN  // Onboard LED

// Variables to track button presses and delay
int pressCount = 0;
const int delayLevels[] = { 50, 100, 500 };  // Three levels of delay
int currentDelay = delayLevels[0];

unsigned long previousMillis = 0;
bool ledState = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up as no external pull-up is connected
}

void loop() {
  // Check if button is pressed
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);  // Debounce delay
    while (digitalRead(BUTTON_PIN) == LOW)
      ;         // Wait until button is released
    delay(5);  // Debounce delay

    // Increment press count and loop through delay levels
    pressCount = (pressCount + 1) % 3;
    currentDelay = delayLevels[pressCount];
  }

  // Non-blocking LED blinking using millis()
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= currentDelay) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}
