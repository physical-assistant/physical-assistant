#include <DFRobot_MAX30102.h>
#include <Wire.h>
#include <TM1637Display.h>

#define CLK 2
#define DIO 3

TM1637Display display(CLK, DIO);

DFRobot_MAX30102 particleSensor;

// declare global variable for button1
bool lastB7 = HIGH;
bool lastB8 = HIGH;
bool lastB9 = HIGH;

// ----------------- CLOCK VARIABLES -----------------
int clockHours = 4;
int clockMinutes = 59;
int clockSeconds = 0;
unsigned long clockPreviousMillis = 0;
bool colonOn = true;

// ----------------- AUTOMATIC WORKOUT SCHEDULE -----------------
struct WorkoutTime {
  int hour;
  int minute;
  bool triggered;  // prevents multiple triggers in same minute
};

// Define your workout times here (24-hour format)
WorkoutTime workoutSchedule[] = {
  {11, 0, false},   // 8:00 AM
  {2, 0, false},  // 12:00 PM
  {5, 0, false}, // 5:30 PM
};

const int NUM_WORKOUTS = sizeof(workoutSchedule) / sizeof(WorkoutTime);

// ----------------- WHITE LED CHASE -----------------
const unsigned long WHITE_STEP_INTERVAL = 1000;  // ms between steps
unsigned long lastWhiteStepTime = 0;
int whiteStep = 0;  // 0,1,2 for LED1, LED2, LED3
bool whiteLEDActive = false;  // controls if white LED chase is running
int whiteStartButton = 0;  // which button started the white LEDs (7, 8, or 9)


// ----------------- ACCELERATION VARIABLES -----------------
float totalAccel = 0.0;        // |A| smoothed magnitude
float dynamicAccel = 0.0;      // |A| - |A_rest|
float restingMagnitude = 0.0;  // |A| when the device is still

// Activity index (0–100, from dynamicAccel)
float dynAvg = 0.0;            // smoothed dynamic acceleration
int activityIndex = 0;         // 0–100

// Noise floor for motion (to ignore tiny noise at rest)
const float DYN_NOISE_FLOOR = 0.5;   // m/s^2, tweak 0.3–0.7 if needed

// ----------------- LED PINS -----------------
const int WHITE_LED1 = 11;
const int WHITE_LED2 = 12;
const int WHITE_LED3 = 13;

// ----------------- ACCELEROMETER CALIBRATION -----------------
const float RestX = 1.403;
const float RestY = 1.399;
const float RestZ = 1.461;
const float Sensitivity = 0.166; // V/g

// ----------------- HEART RATE VARIABLES -----------------
const int LOW_HR_THRESHOLD = 50;
const int HIGH_HR_THRESHOLD = 110;
const unsigned long NO_BEAT_MS = 3000;

const float ALPHA = 0.95;
long irBaseline = 0;
bool aboveThreshold = false;
unsigned long lastBeatTime = 0;

int BEAT_THRESHOLD = 1000;
const int MIN_BEAT_MS = 300;
const int MAX_BEAT_MS = 2000;

const int BPM_BUF_SIZE = 4;
float bpmBuffer[BPM_BUF_SIZE];
int bpmIndex = 0;
int bpmCount = 0;
float currentBPM = 0;

// ----------------- LED TIMER -----------------
const int BLUE_LED_PIN = 10;
const unsigned long CYCLE_TIME = 2UL * 60 * 1000;
const unsigned long ON_TIME = 30UL * 1000;
const unsigned long OFF_TIME = CYCLE_TIME - ON_TIME;

unsigned long previousMillis = 0;
bool ledState = false;
bool blueLEDActive = false;  // controls if blue LED timer is running
int blueStartButton = 0;  // which button started the blue LED (7, 8, or 9)

// ----------------- BUTTONS -----------------
const int BUTTON1_PIN = 8;
const int BUTTON2_PIN = 9;
const int LED_PIN = 13;

bool button1State = false;
bool button2State = false;

bool lastButton1Reading = HIGH;
bool lastButton2Reading = HIGH;
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
const unsigned long debounceDelay = 50;

// ----------------- DATA OUTPUT -----------------
unsigned long lastOutputTime = 0;
const unsigned long OUTPUT_INTERVAL = 1000;  // 1s

// ----------------- ACCELEROMETER -----------------
const int xPin = A0;
const int yPin = A1;
const int zPin = A3;
const int SAMPLES = 10;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float accelXSmooth = 0.0;
float accelYSmooth = 0.0;
float accelZSmooth = 0.0;
const float ACCEL_ALPHA = 0.8;

// ----------------- FALL VARIABLES -----------------
const float FREE_FALL_THRESHOLD = 5.0;
const float IMPACT_THRESHOLD = 20.0;
const unsigned long FALL_WINDOW = 1500;
const unsigned long FALL_COOLDOWN = 5000;

bool isFalling = false;
unsigned long fallStartTime = 0;
int fallDetected = 0;
unsigned long fallDetectedTime = 0;

// ----------------- FUNCTION PROTOTYPES -----------------
float readAveragedVoltage(int pin, int samples);
float smoothValue(float newVal, float prevVal, float alpha);
void readAccelerometer();
void detectFall(float ax, float ay, float az);
void handleButtons();
void handleBlueLEDTimer();
void readHeartbeat();
void outputDataForWebApp();
void handleWhiteLEDChase();
void updateClock();
void setTime(int h, int m, int s);
void stopWhiteLEDs();
void stopBlueLED();
void startWhiteLEDs(int buttonNum);
void startBlueLED(int buttonNum);
void checkScheduledWorkouts();

// =======================================================
// ======================= SETUP =========================
// =======================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  display.setBrightness(0x0f);
  
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  // Initialize MAX30102
  while (!particleSensor.begin()) {
    delay(1000);
  }

  particleSensor.sensorConfiguration(
    80,
    SAMPLEAVG_8,
    MODE_RED_IR,
    SAMPLERATE_100,
    PULSEWIDTH_411,
    ADCRANGE_16384
  );

  pinMode(WHITE_LED1, OUTPUT);
  pinMode(WHITE_LED2, OUTPUT);
  pinMode(WHITE_LED3, OUTPUT);
  
  // Make sure white LEDs start OFF
  digitalWrite(WHITE_LED1, LOW);
  digitalWrite(WHITE_LED2, LOW);
  digitalWrite(WHITE_LED3, LOW);

  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // ----------------- CALIBRATE RESTING MAGNITUDE -----------------
  delay(500);
  float sumMag = 0;

  for (int i = 0; i < 20; i++) {
    readAccelerometer();  // updates accelX, accelY, accelZ
    float mag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    sumMag += mag;
    delay(50);
  }

  restingMagnitude = sumMag / 20.0;
  dynAvg = 0.0;  // start dynamic average at 0 (rest)
  Serial.print("Resting magnitude calibrated to: ");
  Serial.println(restingMagnitude, 2);
  
  // Print scheduled workout times
  Serial.println("Scheduled workout times:");
  for (int i = 0; i < NUM_WORKOUTS; i++) {
    Serial.print("  ");
    if (workoutSchedule[i].hour < 10) Serial.print("0");
    Serial.print(workoutSchedule[i].hour);
    Serial.print(":");
    if (workoutSchedule[i].minute < 10) Serial.print("0");
    Serial.println(workoutSchedule[i].minute);
  }
}

// =======================================================
// ======================= LOOP ==========================
// =======================================================

void loop() {

  handleWhiteLEDChase();
  readAccelerometer();
  handleButtons();
  handleBlueLEDTimer();
  readHeartbeat();
  outputDataForWebApp();
  updateClock();
  checkScheduledWorkouts();  // Check if it's time for automatic workout
  
  int b7 = digitalRead(7);
  int b8 = digitalRead(8);
  int b9 = digitalRead(9);

  if (b7 == LOW && lastB7 == HIGH) {
    Serial.println("BUTTON1");
    handleButtonPress(7);
  }
  
  if (b8 == LOW && lastB8 == HIGH) {
    Serial.println("BUTTON2");
    handleButtonPress(8);
  }
  
  if (b9 == LOW && lastB9 == HIGH) {
    Serial.println("BUTTON3");
    handleButtonPress(9);
  }

  lastB7 = b7;
  lastB8 = b8;
  lastB9 = b9;
  delay(20);
}

// =======================================================
// ============== SCHEDULED WORKOUT CHECK ================
// =======================================================

void checkScheduledWorkouts() {
  for (int i = 0; i < NUM_WORKOUTS; i++) {
    // Check if current time matches scheduled time
    if (clockHours == workoutSchedule[i].hour && 
        clockMinutes == workoutSchedule[i].minute) {
      
      // Only trigger once per minute
      if (!workoutSchedule[i].triggered) {
        workoutSchedule[i].triggered = true;
        
        Serial.print("SCHEDULED WORKOUT STARTED at ");
        if (clockHours < 10) Serial.print("0");
        Serial.print(clockHours);
        Serial.print(":");
        if (clockMinutes < 10) Serial.print("0");
        Serial.println(clockMinutes);
        
        // Start both LED sequences (use button 0 to indicate automatic start)
        if (!whiteLEDActive) {
          startWhiteLEDs(0);  // 0 = automatic start
        }
        if (!blueLEDActive) {
          startBlueLED(0);  // 0 = automatic start
        }
      }
    } else {
      // Reset trigger when we're past this minute
      workoutSchedule[i].triggered = false;
    }
  }
}

// =======================================================
// ================ BUTTON PRESS HANDLER =================
// =======================================================

void handleButtonPress(int buttonNum) {
  // Check WHITE LED control
  if (!whiteLEDActive) {
    // White LEDs are off - start them with this button
    startWhiteLEDs(buttonNum);
  } else if (whiteStartButton == buttonNum) {
    // Same button that started white LEDs - stop them
    stopWhiteLEDs();
  } else {
    // Different button - reset and restart white LEDs
    stopWhiteLEDs();
    startWhiteLEDs(buttonNum);
  }
  
  // Check BLUE LED control
  if (!blueLEDActive) {
    // Blue LED is off - start it with this button
    startBlueLED(buttonNum);
  } else if (blueStartButton == buttonNum) {
    // Same button that started blue LED - stop it
    stopBlueLED();
  } else {
    // Different button - reset and restart blue LED
    stopBlueLED();
    startBlueLED(buttonNum);
  }
}

// =======================================================
// ================ START/STOP FUNCTIONS =================
// =======================================================

void startWhiteLEDs(int buttonNum) {
  whiteLEDActive = true;
  whiteStartButton = buttonNum;
  lastWhiteStepTime = millis();
  whiteStep = 0;  // Reset to first LED
  if (buttonNum == 0) {
    Serial.println("White LED chase started (automatic)");
  } else {
    Serial.println("White LED chase started");
  }
}

void stopWhiteLEDs() {
  whiteLEDActive = false;
  whiteStartButton = 0;
  digitalWrite(WHITE_LED1, LOW);
  digitalWrite(WHITE_LED2, LOW);
  digitalWrite(WHITE_LED3, LOW);
  Serial.println("White LED chase stopped");
}

void startBlueLED(int buttonNum) {
  blueLEDActive = true;
  blueStartButton = buttonNum;
  previousMillis = millis();
  ledState = false;  // Start in OFF state
  digitalWrite(BLUE_LED_PIN, LOW);
  if (buttonNum == 0) {
    Serial.println("Blue LED timer started (automatic)");
  } else {
    Serial.println("Blue LED timer started");
  }
}

void stopBlueLED() {
  blueLEDActive = false;
  blueStartButton = 0;
  digitalWrite(BLUE_LED_PIN, LOW);
  ledState = false;
  Serial.println("Blue LED timer stopped");
}

// =======================================================
// ======================= CLOCK =========================
// =======================================================

void updateClock() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - clockPreviousMillis >= 1000) {
    clockPreviousMillis = currentMillis;
    
    clockSeconds++;
    if (clockSeconds >= 60) {
      clockSeconds = 0;
      clockMinutes++;
      if (clockMinutes >= 60) {
        clockMinutes = 0;
        clockHours++;
        if (clockHours >= 24) {
          clockHours = 0;
        }
      }
    }
    
    colonOn = !colonOn;
    
    int timeValue = (clockHours * 100) + clockMinutes;
    uint8_t colonMask = colonOn ? 0b01000000 : 0b00000000;
    display.showNumberDecEx(timeValue, colonMask, true);
  }
}

void setTime(int h, int m, int s) {
  clockHours = h;
  clockMinutes = m;
  clockSeconds = s;
}

// =======================================================
// ============== ACCELEROMETER FUNCTIONS ================
// =======================================================

float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  return (sum / (float)samples) * (5.0 / 1023.0);
}

float smoothValue(float newVal, float prevVal, float alpha) {
  return alpha * prevVal + (1.0 - alpha) * newVal;
}

void handleWhiteLEDChase() {
  // Only run if white LED chase is active
  if (!whiteLEDActive) {
    return;
  }
  
  unsigned long now = millis();

  // Only update when enough time has passed
  if (now - lastWhiteStepTime >= WHITE_STEP_INTERVAL) {
    lastWhiteStepTime = now;

    // Advance to next step in the chase
    whiteStep = (whiteStep + 1) % 3;  // 0 → 1 → 2 → 0 ...

    // Turn the correct LED on, others off
    digitalWrite(WHITE_LED1, (whiteStep == 0) ? HIGH : LOW);
    digitalWrite(WHITE_LED2, (whiteStep == 1) ? HIGH : LOW);
    digitalWrite(WHITE_LED3, (whiteStep == 2) ? HIGH : LOW);
  }
}


void readAccelerometer() {
  float xVoltage = readAveragedVoltage(xPin, SAMPLES);
  float yVoltage = readAveragedVoltage(yPin, SAMPLES);
  float zVoltage = readAveragedVoltage(zPin, SAMPLES);

  float g_x = (xVoltage - RestX) / Sensitivity;
  float g_y = (yVoltage - RestY) / Sensitivity;
  float g_z = (zVoltage - RestZ) / Sensitivity;

  float rawX = g_x * 9.81;
  float rawY = g_y * 9.81;
  float rawZ = g_z * 9.81;

  accelXSmooth = smoothValue(rawX, accelXSmooth, ACCEL_ALPHA);
  accelYSmooth = smoothValue(rawY, accelYSmooth, ACCEL_ALPHA);
  accelZSmooth = smoothValue(rawZ, accelZSmooth, ACCEL_ALPHA);

  accelX = accelXSmooth;
  accelY = accelYSmooth;
  accelZ = accelZSmooth;

  // === FALL DETECTION (use raw, unsmoothed) ===
  detectFall(rawX, rawY, rawZ);

  // === TOTAL |A| (smoothed) ===
  totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // === DYNAMIC ACCELERATION ===
  dynamicAccel = totalAccel - restingMagnitude;

  // Noise floor: ignore tiny "movement" (sensor noise / micro-vibration)
  if (dynamicAccel < DYN_NOISE_FLOOR) {
    dynamicAccel = 0;
  }

  // === DYNAMIC AVERAGE (approx "last ~few seconds" via EMA) ===
  const float ACT_ALPHA = 0.9;   // smoothing for activity index
  dynAvg = ACT_ALPHA * dynAvg + (1.0 - ACT_ALPHA) * dynamicAccel;

  // If the smoothed motion is tiny, treat as full rest
  if (dynAvg < DYN_NOISE_FLOOR) {
    dynAvg = 0;
  }

  // === ACTIVITY INDEX (0–100) ===
  // Assume dynAvg ~ 0–6 m/s^2 in realistic movement, map 0–6 → 0–100
  float idx = (dynAvg / 6.0f) * 100.0f;
  if (idx < 0) idx = 0;
  if (idx > 100) idx = 100;
  activityIndex = (int)(idx + 0.5f);  // round to nearest int
}

// =======================================================
// ================= FALL DETECTION ======================
// =======================================================
void detectFall(float ax, float ay, float az) {
  float instTotal = sqrt(ax * ax + ay * ay + az * az);
  unsigned long now = millis();

  // Reset fall after cooldown
  if (fallDetected == 1 && (now - fallDetectedTime) > FALL_COOLDOWN) {
    fallDetected = 0;
    isFalling = false;
  }

  if (fallDetected == 1) return;

  // Phase 1: Detect free-fall (very low acceleration)
  if (instTotal < FREE_FALL_THRESHOLD && !isFalling) {
    isFalling = true;
    fallStartTime = now;
  }

  // Phase 2: Detect impact after free-fall
  if (isFalling) {
    unsigned long fallDuration = now - fallStartTime;

    // Need BOTH: high impact AND some recent activity
    if (instTotal > IMPACT_THRESHOLD && fallDuration < FALL_WINDOW && activityIndex >= 3) {
      fallDetected = 1;
      fallDetectedTime = now;
      isFalling = false;
    }

    // Timeout - no impact detected
    if (fallDuration > FALL_WINDOW) {
      isFalling = false;
    }
  }
}

// =======================================================
// ==================== BUTTONS ==========================
// =======================================================

void handleButtons() {
  bool b1 = digitalRead(BUTTON1_PIN);
  bool b2 = digitalRead(BUTTON2_PIN);

  if (b1 != lastButton1Reading) lastButton1Time = millis();
  if ((millis() - lastButton1Time) > debounceDelay) {
    if (b1 == LOW && lastButton1Reading == HIGH) {
      button1State = !button1State;
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton1Reading = b1;

  if (b2 != lastButton2Reading) lastButton2Time = millis();
  if ((millis() - lastButton2Time) > debounceDelay) {
    if (b2 == LOW && lastButton2Reading == HIGH) {
      button2State = !button2State;
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }
  }
  lastButton2Reading = b2;
}

// =======================================================
// ==================== BLUE LED TIMER ===================
// =======================================================

void handleBlueLEDTimer() {
  // Only run if blue LED timer is active
  if (!blueLEDActive) {
    return;
  }
  
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

// =======================================================
// ==================== HEART SENSOR =====================
// =======================================================

void readHeartbeat() {
  int irValue = particleSensor.getIR();
  unsigned long now = millis();

  if (irBaseline == 0) irBaseline = irValue;
  else irBaseline = (long)(ALPHA * irBaseline + (1.0 - ALPHA) * irValue);

  long acValue = irValue - irBaseline;

  static long maxAC = 0;
  maxAC = max(maxAC * 0.98, abs(acValue) * 0.02);
  BEAT_THRESHOLD = max(500, (int)(maxAC * 0.5));

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

      currentBPM = avgBPM;
    }

    lastBeatTime = now;
  }

  aboveThreshold = isAbove;

  if (lastBeatTime != 0 && (now - lastBeatTime) > NO_BEAT_MS) {
    lastBeatTime = 0;
    bpmCount = 0;
    currentBPM = 0;
  }
}

// =======================================================
// ==================== SERIAL OUTPUT ====================
// =======================================================

void outputDataForWebApp() {
  unsigned long now = millis();

  if (now - lastOutputTime >= OUTPUT_INTERVAL) {
    lastOutputTime = now;

    Serial.print("HR:");
    Serial.print((int)currentBPM);

    Serial.print(",ACT:");
    Serial.print(activityIndex);          // 0–100

    Serial.print(",DYN:");
    Serial.print(dynamicAccel, 2);        // debug / optional

    Serial.print(",MAG:");
    Serial.print(totalAccel, 2);          // debug / optional

    Serial.print(",FALL:");
    Serial.println(fallDetected);
  }
}