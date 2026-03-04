#include <Arduino.h>
#include <EEPROM.h>
#include <EEPROMLayout.h>
#include <ServoControl.h>
#include <MagControl.h>

#define BTN_L            8
#define BTN_R            9
#define LIMIT_PIN        4
#define SOLENOID_PIN     2
#define DEBOUNCE_MS     75
#define LIMIT_DEBOUNCE   2
#define PULSE_MS         50
#define FIRE_INTERVAL_MS 200
#define FIRE_TEST_MS    1000  // observation window for single-fire test (ms)

// ── Steering tuning parameters ──
float Kp                = 5.0;    // proportional gain
float target_heading    = 45.0;   // target heading in degrees (relative to tare)
float heading_tolerance = 2.0;    // tolerance in degrees

Servo        servo;
ServoControl sc(servo);
MagControl   mc;

// Heading / steering state
bool streaming = false;
bool steering  = false;

// Solenoid state
unsigned long solTimer  = 0;
bool          solPulsed = false;

// Button debounce state
bool          btnL_raw = HIGH, btnL_stable = HIGH;
unsigned long btnL_debounce = 0;
bool          btnR_raw = HIGH, btnR_stable = HIGH;
unsigned long btnR_debounce = 0;

// Limit switch debounce state (GPIO 4)
bool          limitRaw = HIGH, limitStable = HIGH;
unsigned long limitDebounce = 0;

// Single-fire test state
bool          firing    = false;
unsigned long fireStart = 0;
int           limitCount = 0;

// Serial buffer for numeric entry
char    serialBuf[16];
uint8_t serialBufIdx    = 0;
bool    serialSettingKp = false;

// ── Helpers ──

void printHelp() {
  Serial.println(F("=== Commands ==="));
  Serial.println(F("-- Servo calibration --"));
  Serial.println(F("  l      - left  (button 1)"));
  Serial.println(F("  r      - right (button 2)"));
  Serial.println(F("  b      - both  (button 3 / store & advance)"));
  Serial.println(F("  s      - print status"));
  Serial.println(F("  x      - clear all EEPROM cal data"));
  Serial.println(F("-- Mag calibration --"));
  Serial.println(F("  m      - start mag calibration (rotate slowly 360)"));
  Serial.println(F("  d      - finish & save mag calibration"));
  Serial.println(F("-- Heading --"));
  Serial.println(F("  a      - stream heading angle (any key to stop)"));
  Serial.println(F("  t      - tare heading to zero"));
  Serial.println(F("-- Steering --"));
  Serial.println(F("  f      - fire solenoid once, count limit switch triggers (2 s)"));
  Serial.println(F("  g      - start Kp steering toward target"));
  Serial.println(F("  p      - stop steering"));
  Serial.println(F("  <num>  - set new target heading (e.g. 90.5)"));
  Serial.println(F("  k      - set Kp gain (then type value + enter)"));
  Serial.println(F("-- Buttons --"));
  Serial.println(F("  GPIO 8 - tare heading"));
  Serial.println(F("  GPIO 9 - toggle steering + solenoid on/off"));
  Serial.println(F("  h      - print this help"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("servo pos:    ")); Serial.println(heading);
  Serial.print(F("center:       ")); Serial.println(servo_center);
  Serial.print(F("left limit:   ")); Serial.println(servo_left_limit);
  Serial.print(F("right limit:  ")); Serial.println(servo_right_limit);
  Serial.print(F("servo valid:  ")); Serial.println(valid ? "yes" : "no");
  Serial.print(F("mag cal:      ")); Serial.println(mc.isCalibrated() ? "yes" : "no");
  Serial.print(F("steering:     ")); Serial.println(steering ? "on" : "off");
  Serial.print(F("target hdg:   ")); Serial.println(target_heading);
  Serial.print(F("Kp:           ")); Serial.println(Kp);
  Serial.print(F("tolerance:    ")); Serial.println(heading_tolerance);
  Serial.println(F("--------------"));
}

void clearAllCal() {
  sc.clearLocation(CENTER_LOCATION);
  sc.clearLocation(LEFT_LOCATION);
  sc.clearLocation(RIGHT_LOCATION);
  mc.clearCal();
  EEPROM.put(STEER_VALID_LOCATION, (uint8_t)0x00);
  Serial.println(F("All EEPROM cal data cleared. Reset board to recalibrate."));
}

// ── Steering control ──

void steerUpdate() {
  float current = mc.heading();
  float error   = target_heading - current;

  Serial.print(current);
  Serial.print(F("  "));
  Serial.print(error);

  // wrap error to -180..+180
  if (error >  180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;

  // within tolerance — hold center
  if (fabs(error) <= heading_tolerance) {
    servo.writeMicroseconds(servo_center);
    Serial.println();
    return;
  }

  float   output    = Kp * error;
  int16_t servo_cmd = servo_center - (int16_t)output;

  Serial.print(F("  "));
  Serial.print(servo_cmd);

  // clamp to calibrated limits
  if (servo_cmd < servo_left_limit && servo_left_limit < servo_center)
    servo_cmd = servo_left_limit;
  else if (servo_cmd < servo_center - 200)
    servo_cmd = servo_center - 200;

  if (servo_cmd > servo_right_limit && servo_right_limit > servo_center)
    servo_cmd = servo_right_limit;
  else if (servo_cmd > servo_center + 200)
    servo_cmd = servo_center + 200;

  Serial.print(F("  "));
  Serial.print(servo_cmd);
  Serial.println();

  servo.writeMicroseconds(servo_cmd);
}

// ── Setup & Loop ──

void setup() {
  Serial.begin(115200);
  ledTimerInit_1ms();

  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  pinMode(BTN_L, INPUT_PULLUP);
  pinMode(BTN_R, INPUT_PULLUP);

  sc.enter();
  mc.enter();   // inits Wire, mag, loads mag cal from EEPROM

  // Load steering params from EEPROM
  uint8_t steer_magic;
  EEPROM.get(STEER_VALID_LOCATION, steer_magic);
  if (steer_magic == 0xC4) {
    EEPROM.get(KP_LOCATION,      Kp);
    EEPROM.get(HEADING_LOCATION, target_heading);
    Serial.println(F("Steering params loaded from EEPROM."));
  }

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  printHelp();
  printStatus();
}

void loop() {
  unsigned long now = millis();

  // ── Button debounce ──
  bool reading;

  reading = digitalRead(BTN_L);
  if (reading != btnL_raw) { btnL_raw = reading; btnL_debounce = now; }
  bool btnL_pressed = false;
  if ((now - btnL_debounce) >= DEBOUNCE_MS && reading != btnL_stable) {
    btnL_stable = reading;
    if (btnL_stable == LOW) btnL_pressed = true;
  }

  reading = digitalRead(BTN_R);
  if (reading != btnR_raw) { btnR_raw = reading; btnR_debounce = now; }
  bool btnR_pressed = false;
  if ((now - btnR_debounce) >= DEBOUNCE_MS && reading != btnR_stable) {
    btnR_stable = reading;
    if (btnR_stable == LOW) btnR_pressed = true;
  }

  // Limit switch debounce (GPIO 4) — count triggers during firing test
  reading = digitalRead(LIMIT_PIN);
  if (reading != limitRaw) { limitRaw = reading; limitDebounce = now; }
  if ((now - limitDebounce) >= LIMIT_DEBOUNCE && reading != limitStable) {
    limitStable = reading;
    if (limitStable == LOW && (firing || steering)) limitCount++;
  }

  // GPIO 8 → tare
  if (btnL_pressed) mc.tare();

  // GPIO 9 → toggle steering + solenoid
  if (btnR_pressed) {
    if (!steering) {
      if (!mc.isCalibrated()) {
        Serial.println(F("Mag not calibrated."));
      } else if (!valid) {
        Serial.println(F("Servo not calibrated."));
      } else {
        steering   = true;
        solPulsed  = false;
        limitCount = 0;
        Serial.print(F("Steering ON. Target="));
        Serial.println(target_heading);
      }
    } else {
      steering = false;
      digitalWrite(SOLENOID_PIN, LOW);
      solPulsed = false;
      servo.writeMicroseconds(servo_center);
      Serial.print(F("Steering stopped. | Limit switch triggers: "));
      Serial.print(limitCount);
      Serial.print("  Distance (in): ");
      Serial.println((double)limitCount * 1.16);
    }
  }

  mc.update();

  // ── Live heading stream ──
  if (streaming) {
    if (mc.isCalibrated()) {
      Serial.print(F("heading: "));
      Serial.println(mc.heading(), 1);
    }
    delay(100);
    if (Serial.available()) {
      Serial.read();
      streaming = false;
      Serial.println(F("Streaming stopped."));
    }
    return;
  }

  // ── Kp steering + solenoid loop ──
  if (steering) {
    if (!solPulsed) {
      digitalWrite(SOLENOID_PIN, HIGH);
      solPulsed = true;
      solTimer  = now;
    }
    if (now - solTimer >= PULSE_MS)         digitalWrite(SOLENOID_PIN, LOW);
    if (now - solTimer >= FIRE_INTERVAL_MS) solPulsed = false;

    steerUpdate();
    delay(20);

    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p') {
        steering = false;
        digitalWrite(SOLENOID_PIN, LOW);
        solPulsed = false;
        servo.writeMicroseconds(servo_center);
        Serial.print(F("Steering stopped. | Limit switch triggers: "));
        Serial.print(limitCount);
        Serial.print("  Distance (in): ");
        Serial.println((double)limitCount * 1.16);
      }
    }
    return;
  }

  // ── Single-fire test ──
  if (firing) {
    if (now - fireStart >= PULSE_MS)      digitalWrite(SOLENOID_PIN, LOW);
    if (now - fireStart >= FIRE_TEST_MS) {
      firing = false;
      digitalWrite(SOLENOID_PIN, LOW);
      Serial.print(F("Fire complete | Limit switch triggers: "));
      Serial.print(limitCount);
      Serial.print("  Distance (in): ");
      Serial.println((double)limitCount * 1.16);
    }
    return;  // ignore all input during the 2-second window
  }

  if (!Serial.available()) return;

  char c = Serial.read();

  // Numeric input → buffer for heading or Kp entry
  if (c == '-' || c == '.' || isdigit((unsigned char)c)) {
    if (serialBufIdx < sizeof(serialBuf) - 1) serialBuf[serialBufIdx++] = c;
    return;
  }
  if ((c == '\n' || c == '\r') && serialBufIdx > 0) {
    serialBuf[serialBufIdx] = '\0';
    serialBufIdx = 0;
    float val = atof(serialBuf);
    if (serialSettingKp) {
      serialSettingKp = false;
      Kp = val;
      EEPROM.put(KP_LOCATION,          Kp);
      EEPROM.put(STEER_VALID_LOCATION,  (uint8_t)0xC4);
      Serial.print(F("Kp set to: "));
      Serial.println(Kp);
    } else {
      target_heading = val;
      EEPROM.put(HEADING_LOCATION,      target_heading);
      EEPROM.put(STEER_VALID_LOCATION,  (uint8_t)0xC4);
      Serial.print(F("Target heading set to: "));
      Serial.println(target_heading);
    }
    return;
  }
  serialBufIdx    = 0;
  serialSettingKp = false;

  uint8_t button = 0;

  switch (c) {
    case 'l': button = 1; break;
    case 'r': button = 2; break;
    case 'b': button = 3; break;
    case 's': printStatus(); return;
    case 'x': clearAllCal(); return;
    case 'm': mc.startCal();  return;
    case 'd': mc.finishCal(); return;
    case 'a':
      if (!mc.isCalibrated()) {
        Serial.println(F("Mag not calibrated. Run 'm' then 'd' first."));
      } else {
        streaming = true;
        Serial.println(F("Streaming heading. Send any key to stop."));
      }
      return;
    case 't':
      if (!mc.isCalibrated()) {
        Serial.println(F("Mag not calibrated. Run 'm' then 'd' first."));
      } else {
        mc.tare();
      }
      return;
    case 'f': // single-fire test
      firing     = true;
      limitCount = 0;
      fireStart  = now;
      digitalWrite(SOLENOID_PIN, HIGH);
      Serial.println(F("Fired. Counting limit switch triggers for 1 s..."));
      return;
    case 'g': // start steering
      if (!mc.isCalibrated()) {
        Serial.println(F("Mag not calibrated first."));
        return;
      }
      if (!valid) {
        Serial.println(F("Servo not calibrated first."));
        return;
      }
      steering   = true;
      limitCount = 0;
      Serial.print(F("Steering ON. Target="));
      Serial.print(target_heading);
      Serial.print(F(" Tol="));
      Serial.print(heading_tolerance);
      Serial.print(F(" Kp="));
      Serial.println(Kp);
      Serial.println(F("Send 'p' to stop."));
      return;
    case 'p':
      Serial.println(F("Steering already off."));
      return;
    case 'k':
      serialSettingKp = true;
      Serial.print(F("Current Kp="));
      Serial.print(Kp);
      Serial.println(F(". Enter new value:"));
      return;
    case 'h': printHelp(); return;
    case '\n':
    case '\r': return;
    default:
      Serial.print(F("Unknown command: "));
      Serial.println(c);
      return;
  }

  sc.update(button);
  printStatus();
}
