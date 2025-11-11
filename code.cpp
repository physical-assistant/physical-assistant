#include <DFRobot_MAX30102.h>
#include <Wire.h>

DFRobot_MAX30102 particleSensor;

// ---- Configurable thresholds ----
const int LOW_HR_THRESHOLD      = 50;     // bpm
const int HIGH_HR_THRESHOLD     = 110;    // bpm
const unsigned long NO_BEAT_MS  = 3000;   // 3 seconds without beat -> warning

// ---- Beat detection variables ----
const float ALPHA = 0.95;        // Smoothing factor for baseline
long irBaseline = 0;             // Smoothed DC component
bool aboveThreshold = false;     // Track threshold crossing
unsigned long lastBeatTime = 0;  // Time of last detected beat (ms)

const int BEAT_THRESHOLD = 2000; // Minimum AC amplitude to count as beat
const int MIN_BEAT_MS    = 300;  // 200 bpm upper limit
const int MAX_BEAT_MS    = 2000; // 30 bpm lower limit

const int BPM_BUF_SIZE = 4;
float bpmBuffer[BPM_BUF_SIZE];
int bpmIndex = 0;
int bpmCount = 0;




// blue LED timer:

const int BLUE_LED_PIN = 13;
const unsigned long CYCLE_TIME = 5UL * 60 * 1000;  // 5 minutes
const unsigned long ON_TIME = 30UL * 1000;          // 30 seconds
const unsigned long OFF_TIME = CYCLE_TIME - ON_TIME; // 4 min 30 sec

unsigned long previousMillis = 0;
bool ledState = false;  // false = OFF, true = ON



// 1 

// Pin definitions
const int BUTTON1_PIN = 8;  // First button connected to pin D8
const int BUTTON2_PIN = 9;  // Second button connected to pin D9
const int LED_PIN = 13;      // Built-in LED for visual feedback

// Button state variables
bool button1State = false;  // false = OFF, true = ON
bool button2State = false;

// Variables for debouncing
bool lastButton1Reading = HIGH;
bool lastButton2Reading = HIGH;
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
const unsigned long debounceDelay = 50;  // 50ms debounce time


void setup() {
   // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  // Set button pins as inputs with internal pull-up resistors
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Flash LED to show board is ready
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("=== Two Button System Ready ===");
  Serial.println("Button 1: OFF | Button 2: OFF");
  Serial.println("Press buttons to test...");
  Serial.println();


  
  Serial.begin(115200);
  Wire.begin();

  while (!particleSensor.begin()) {
    Serial.println("MAX30102 not found!");
    delay(1000);
  }

  particleSensor.sensorConfiguration(
    60,               // LED brightness
    SAMPLEAVG_8,      // Sample average
    MODE_MULTILED,    // IR + Red
    SAMPLERATE_400,   // Sample rate
    PULSEWIDTH_411,   // Pulse width
    ADCRANGE_16384    // ADC range
  );

  Serial.println("MAX30102 initialized successfully!");
  Serial.println("Place finger on sensor...");


  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);  // Start with LED off

  Serial.begin(9600);
  Serial.println("Blue LED Timer Started");
  Serial.println("Cycle: 5 min total (30s ON, 4m 30s OFF)");
}

void loop() {
  
  // Read current button states (RAW values for debugging)
  bool button1Reading = digitalRead(BUTTON1_PIN);
  bool button2Reading = digitalRead(BUTTON2_PIN);
  
  // Print raw values every 2 seconds for debugging
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("Raw readings - D8: ");
    Serial.print(button1Reading ? "HIGH" : "LOW");
    Serial.print(" | D9: ");
    Serial.println(button2Reading ? "HIGH" : "LOW");
    lastPrint = millis();
  }
  
  // Handle Button 1
  if (button1Reading != lastButton1Reading) {
    lastButton1Time = millis();
  }
  
  if ((millis() - lastButton1Time) > debounceDelay) {
    if (button1Reading == LOW && lastButton1Reading == HIGH) {
      // Button 1 was pressed (toggled)
      button1State = !button1State;
      Serial.println(">>> BUTTON 1 PRESSED <<<");
      Serial.print("Button 1 State: ");
      Serial.println(button1State ? "ON" : "OFF");
      
      // Flash LED as feedback
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton1Reading = button1Reading;
  
  // Handle Button 2
  if (button2Reading != lastButton2Reading) {
    lastButton2Time = millis();
  }
  
  if ((millis() - lastButton2Time) > debounceDelay) {
    if (button2Reading == LOW && lastButton2Reading == HIGH) {
      // Button 2 was pressed (toggled)
      button2State = !button2State;
      Serial.println(">>> BUTTON 2 PRESSED <<<");
      Serial.print("Button 2 State: ");
      Serial.println(button2State ? "ON" : "OFF");
      
      // Flash LED as feedback
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton2Reading = button2Reading;


  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - previousMillis;

  // State machine: switch between ON and OFF
  if (!ledState) {
    // LED is currently OFF - waiting for OFF_TIME to complete
    if (elapsed >= OFF_TIME) {
      // Time to turn ON
      digitalWrite(BLUE_LED_PIN, HIGH);
      ledState = true;
      previousMillis = currentMillis;
      Serial.println("LED ON");
    }
  } else {
    // LED is currently ON - waiting for ON_TIME to complete
    if (elapsed >= ON_TIME) {
      // Time to turn OFF
      digitalWrite(BLUE_LED_PIN, LOW);
      ledState = false;
      previousMillis = currentMillis;
      Serial.println("LED OFF");
    }
  }


  int irValue = particleSensor.getIR();
  unsigned long now = millis();

  // Baseline smoothing
  if (irBaseline == 0) {
    irBaseline = irValue;
  } else {
    irBaseline = (long)(ALPHA * irBaseline + (1.0 - ALPHA) * irValue);
  }

  long acValue = irValue - irBaseline;

  // Beat detection
  bool isAbove = (acValue > BEAT_THRESHOLD);
  if (isAbove && !aboveThreshold) {
    unsigned long interval = now - lastBeatTime;

    if (lastBeatTime != 0 && interval > MIN_BEAT_MS && interval < MAX_BEAT_MS) {
      float instantBPM = 60000.0 / interval;

      // Moving average BPM
      bpmBuffer[bpmIndex] = instantBPM;
      bpmIndex = (bpmIndex + 1) % BPM_BUF_SIZE;
      if (bpmCount < BPM_BUF_SIZE) bpmCount++;

      float avgBPM = 0;
      for (int i = 0; i < bpmCount; i++) {
        avgBPM += bpmBuffer[i];
      }
      avgBPM /= bpmCount;

      // Warnings
      if (avgBPM < LOW_HR_THRESHOLD) {
        Serial.print("IR: "); Serial.print(irValue);
        Serial.print("  BPM: "); Serial.print(avgBPM, 1);
        Serial.println("  → WARNING: Low heart rate detected.");
      } else if (avgBPM > HIGH_HR_THRESHOLD) {
        Serial.print("IR: "); Serial.print(irValue);
        Serial.print("  BPM: "); Serial.print(avgBPM, 1);
        Serial.println("  → WARNING: High heart rate detected.");
      } else {
        Serial.print("IR: "); Serial.print(irValue);
        Serial.print("  BPM: "); Serial.print(avgBPM, 1);
        Serial.println("  → Normal range.");
      }
    }

    lastBeatTime = now;
  }

  aboveThreshold = isAbove;

  // No-beat warning
  if (lastBeatTime != 0 && (now - lastBeatTime) > NO_BEAT_MS) {
    Serial.println("WARNING: No reliable pulse detected — check finger/sensor placement.");
    lastBeatTime = 0;
    bpmCount = 0;
  }

  delay(20);
}