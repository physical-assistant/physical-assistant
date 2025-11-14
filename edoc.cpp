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

int BEAT_THRESHOLD = 1000;       // Adaptive threshold (will update dynamically)
const int MIN_BEAT_MS    = 300;  // 200 bpm upper limit
const int MAX_BEAT_MS    = 2000; // 30 bpm lower limit

const int BPM_BUF_SIZE = 4;
float bpmBuffer[BPM_BUF_SIZE];
int bpmIndex = 0;
int bpmCount = 0;
float currentBPM = 0;  // Store current averaged BPM

// ---- Blue LED timer ----
const int BLUE_LED_PIN = 13;
const unsigned long CYCLE_TIME = 5UL * 60 * 1000;  // 5 minutes
const unsigned long ON_TIME = 30UL * 1000;          // 30 seconds
const unsigned long OFF_TIME = CYCLE_TIME - ON_TIME; // 4 min 30 sec
unsigned long previousMillis = 0;
bool ledState = false;

// ---- Buttons ----
const int BUTTON1_PIN = 8;
const int BUTTON2_PIN = 9;
const int LED_PIN = 13;  // Built-in LED feedback

bool button1State = false;
bool button2State = false;

bool lastButton1Reading = HIGH;
bool lastButton2Reading = HIGH;
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
const unsigned long debounceDelay = 50;

// ---- Data output timer ----
unsigned long lastOutputTime = 0;
const unsigned long OUTPUT_INTERVAL = 1000;  // Output every 1 second

// ---- Accelerometer Variables ----
const int xPin = A0;
const int yPin = A1;
const int zPin = A3;
const int SAMPLES = 10;

const float RestVoltage = 1.65;    // Voltage at 0 g (adjust based on your sensor)
const float sensitivity = 0.3;     // Sensitivity in V/g for ADXL335 (300mV/g)

// Global acceleration variables
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

// Smoothing for accelerometer
float accelXSmooth = 0.0;
float accelYSmooth = 0.0;
float accelZSmooth = 0.0;
const float ACCEL_ALPHA = 0.8;  // Smoothing factor

// ---- Fall Detection Variables ----
const float FREE_FALL_THRESHOLD = 4.0;   // m/s^2 (~0.4 g) - total acceleration drops
const float IMPACT_THRESHOLD = 25.0;     // m/s^2 (~2.5 g) - sudden impact
const unsigned long FALL_WINDOW = 1500;  // ms - time window to detect impact after freefall
const unsigned long FALL_COOLDOWN = 5000; // ms - cooldown period after fall detection

bool isFalling = false;
unsigned long fallStartTime = 0;
int fallDetected = 0;  // 0 = no fall, 1 = fall detected
unsigned long fallDetectedTime = 0;

// ---- Function Prototypes ----
float readAveragedVoltage(int pin, int samples);
float smoothValue(float newVal, float prevVal, float alpha);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MAX30102
  while (!particleSensor.begin()) {
    delay(1000);
  }

  particleSensor.sensorConfiguration(
    50,               // LED brightness (stronger signal)
    SAMPLEAVG_8,       // Average samples
    MODE_RED_IR,       // Use both IR + Red
    SAMPLERATE_100,    // Slower sample rate = smoother signal
    PULSEWIDTH_411,    // Long pulse = better range
    ADCRANGE_16384
  );

  // Blue LED setup
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  // Button setup
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Initialize accelerometer baseline
  delay(500);  // Let sensor stabilize
  for (int i = 0; i < 10; i++) {
    readAccelerometer();
    delay(50);
  }
}

void loop() {
  // --- Read accelerometer ---
  readAccelerometer();

  // --- Button handling ---
  handleButtons();

  // --- Blue LED timer ---
  handleBlueLEDTimer();

  // --- Heartbeat detection ---
  readHeartbeat();

  // --- Output data for web app ---
  outputDataForWebApp();

  delay(20); // 50 Hz loop
}

float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  // Convert to voltage (assuming 5V Arduino)
  return (sum / (float)samples) * (5.0 / 1023.0);
}

float smoothValue(float newVal, float prevVal, float alpha) {
  return alpha * prevVal + (1.0 - alpha) * newVal;
}

void readAccelerometer() {
  // Read raw voltages
  float xVoltage = readAveragedVoltage(xPin, SAMPLES);
  float yVoltage = readAveragedVoltage(yPin, SAMPLES);
  float zVoltage = readAveragedVoltage(zPin, SAMPLES);

  // Convert voltages to g (gravity units)
  float g_x = (xVoltage - RestVoltage) / sensitivity;
  float g_y = (yVoltage - RestVoltage) / sensitivity;
  float g_z = (zVoltage - RestVoltage) / sensitivity;

  // Convert g to m/s^2
  float rawAccelX = g_x * 9.81;
  float rawAccelY = g_y * 9.81;
  float rawAccelZ = g_z * 9.81;

  // Apply smoothing
  accelXSmooth = smoothValue(rawAccelX, accelXSmooth, ACCEL_ALPHA);
  accelYSmooth = smoothValue(rawAccelY, accelYSmooth, ACCEL_ALPHA);
  accelZSmooth = smoothValue(rawAccelZ, accelZSmooth, ACCEL_ALPHA);

  // Use smoothed values for output
  accelX = accelXSmooth;
  accelY = accelYSmooth;
  accelZ = accelZSmooth;

  // ---- Fall Detection Logic ----
  detectFall(rawAccelX, rawAccelY, rawAccelZ);
}

void detectFall(float ax, float ay, float az) {
  // Calculate total acceleration magnitude
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  
  unsigned long now = millis();

  // Auto-clear fall detection after cooldown period
  if (fallDetected == 1 && (now - fallDetectedTime) > FALL_COOLDOWN) {
    fallDetected = 0;
    isFalling = false;
  }

  // Don't detect new falls during cooldown
  if (fallDetected == 1) {
    return;
  }

  // Stage 1: Detect freefall (sudden drop in total acceleration)
  if (totalAccel < FREE_FALL_THRESHOLD && !isFalling) {
    isFalling = true;
    fallStartTime = now;
  }

  // Stage 2: Detect impact after freefall
  if (isFalling) {
    unsigned long fallDuration = now - fallStartTime;
    
    // Check for impact within the fall window
    if (totalAccel > IMPACT_THRESHOLD && fallDuration < FALL_WINDOW) {
      fallDetected = 1;
      fallDetectedTime = now;
      isFalling = false;
    }
    
    // Reset if fall window expires without impact
    if (fallDuration > FALL_WINDOW) {
      isFalling = false;
    }
  }
}

void handleButtons() {
  bool button1Reading = digitalRead(BUTTON1_PIN);
  bool button2Reading = digitalRead(BUTTON2_PIN);

  // Debounce button 1
  if (button1Reading != lastButton1Reading) lastButton1Time = millis();
  if ((millis() - lastButton1Time) > debounceDelay) {
    if (button1Reading == LOW && lastButton1Reading == HIGH) {
      button1State = !button1State;
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton1Reading = button1Reading;

  // Debounce button 2
  if (button2Reading != lastButton2Reading) lastButton2Time = millis();
  if ((millis() - lastButton2Time) > debounceDelay) {
    if (button2Reading == LOW && lastButton2Reading == HIGH) {
      button2State = !button2State;
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton2Reading = button2Reading;
}

void handleBlueLEDTimer() {
  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - previousMillis;

  if (!ledState) {
    if (elapsed >= OFF_TIME) {
      digitalWrite(BLUE_LED_PIN, HIGH);
      ledState = true;
      previousMillis = currentMillis;
    }
  } else {
    if (elapsed >= ON_TIME) {
      digitalWrite(BLUE_LED_PIN, LOW);
      ledState = false;
      previousMillis = currentMillis;
    }
  }
}

void readHeartbeat() {
  int irValue = particleSensor.getIR();
  unsigned long now = millis();

  // Baseline smoothing
  if (irBaseline == 0) irBaseline = irValue;
  else irBaseline = (long)(ALPHA * irBaseline + (1.0 - ALPHA) * irValue);

  long acValue = irValue - irBaseline;

  // --- Adaptive threshold ---
  static long maxAC = 0;
  maxAC = max(maxAC * 0.98, abs(acValue) * 0.02);
  BEAT_THRESHOLD = max(500, (int)(maxAC * 0.5)); // never below 500

  // --- Beat detection ---
  bool isAbove = (acValue > BEAT_THRESHOLD);
  if (isAbove && !aboveThreshold) {
    unsigned long interval = now - lastBeatTime;

    if (lastBeatTime != 0 && interval > MIN_BEAT_MS && interval < MAX_BEAT_MS) {
      float instantBPM = 60000.0 / interval;

      bpmBuffer[bpmIndex] = instantBPM;
      bpmIndex = (bpmIndex + 1) % BPM_BUF_SIZE;
      if (bpmCount < BPM_BUF_SIZE) bpmCount++;

      float avgBPM = 0;
      for (int i = 0; i < bpmCount; i++) avgBPM += bpmBuffer[i];
      avgBPM /= bpmCount;

      currentBPM = avgBPM;  // Store for web app output
    }

    lastBeatTime = now;
  }

  aboveThreshold = isAbove;

  // --- No-beat warning ---
  if (lastBeatTime != 0 && (now - lastBeatTime) > NO_BEAT_MS) {
    lastBeatTime = 0;
    bpmCount = 0;
    currentBPM = 0;
  }
}

void outputDataForWebApp() {
  unsigned long now = millis();
  
  // Output data at regular intervals
  if (now - lastOutputTime >= OUTPUT_INTERVAL) {
    lastOutputTime = now;
    
    // Format: HR:78,AX:0.12,AY:-0.05,AZ:1.01,FALL:0
    Serial.print("HR:");
    Serial.print((int)currentBPM);
    Serial.print(",AX:");
    Serial.print(accelX, 2);
    Serial.print(",AY:");
    Serial.print(accelY, 2);
    Serial.print(",AZ:");
    Serial.print(accelZ, 2);
    Serial.print(",FALL:");
    Serial.println(fallDetected);
  }
}
