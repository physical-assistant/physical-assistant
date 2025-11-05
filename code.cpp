// ignore errors, they will disappear when ran in Arduino IDE
#include <Wire.h>
#include "MAX30105.h"    // Use the SparkFun MAX3010x library
#include "heartRate.h"

MAX30105 particleSensor;

const int LED_PIN = 13;
const int BUTTON1_PIN = 8;
const int BUTTON2_PIN = 9;

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

// Blink LED to confirm setup start
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }

    Serial.println("Initializing MAX30102 sensor...");

    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 not found. Check wiring!");
        while (1);
    }

    particleSensor.setup(); // Default: 69 samples/sec, LED power 50mA
    Serial.println("Sensor initialized successfully!");
}

void loop() {
// --- Buttons ---
    bool button1State = !digitalRead(BUTTON1_PIN); // Active LOW
    bool button2State = !digitalRead(BUTTON2_PIN);

    if (button1State) {
        Serial.println("Button 1 pressed!");
        digitalWrite(LED_PIN, HIGH);
    } else if (button2State) {
        Serial.println("Button 2 pressed!");
        digitalWrite(LED_PIN, LOW);
    }

  // --- Sensor readings ---
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print("\tRED: ");
    Serial.println(redValue);

    delay(250); // slow down serial output
}