#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LIS3MDL.h>
#include <ServoControl.h>

#define BTN_L            8
#define BTN_R            9
#define SOLENOID_PIN     2
#define DEBOUNCE_MS     75
#define PULSE_MS       50
#define FIRE_INTERVAL_MS 500

// Mag calibration EEPROM layout (servo data ends at byte 7)
const uint8_t MAG_VALID_LOCATION    = 7;
const uint8_t MAG_OFFSET_X_LOCATION = MAG_VALID_LOCATION    + sizeof(uint8_t);
const uint8_t MAG_OFFSET_Y_LOCATION = MAG_OFFSET_X_LOCATION + sizeof(float);
const uint8_t MAG_RANGE_X_LOCATION  = MAG_OFFSET_Y_LOCATION + sizeof(float);
const uint8_t MAG_RANGE_Y_LOCATION  = MAG_RANGE_X_LOCATION  + sizeof(float);

// Steering params EEPROM layout (after mag data)
const uint8_t STEER_VALID_LOCATION   = MAG_RANGE_Y_LOCATION  + sizeof(float);
const uint8_t KP_LOCATION            = STEER_VALID_LOCATION  + sizeof(uint8_t);
const uint8_t HEADING_LOCATION       = KP_LOCATION           + sizeof(float);

// ── Steering tuning parameters ──
float Kp                = 5.0;    // proportional gain
float target_heading    = 45.0;   // target heading in degrees (relative to tare)
float heading_tolerance = 1.0;    // tolerance in degrees

Servo servo;
ServoControl sc(servo);

LIS3MDL mag;

// Magnetometer calibration state
bool mag_collecting = false;
bool mag_calibrated = false;
int16_t mag_min_x, mag_max_x, mag_min_y, mag_max_y;
float offset_x, offset_y;
float range_x, range_y;

// Heading state
float tare_offset   = 0.0;  // heading at tare time (degrees)
bool  streaming     = false; // live heading stream active
bool  steering      = false; // Kp steering loop active

// Solenoid state
unsigned long solTimer  = 0;
bool          solPulsed = false;

// Button debounce state
bool          btnL_raw = HIGH, btnL_stable = HIGH;
unsigned long btnL_debounce = 0;
bool          btnR_raw = HIGH, btnR_stable = HIGH;
unsigned long btnR_debounce = 0;

// Serial buffer for numeric entry
char    serialBuf[16];
uint8_t serialBufIdx = 0;
bool    serialSettingKp = false;

void printHelp() {
  Serial.println(F("=== Commands ==="));
  Serial.println(F("-- Servo calibration --"));
  Serial.println(F("  l  - left  (button 1)"));
  Serial.println(F("  r  - right (button 2)"));
  Serial.println(F("  b  - both  (button 3 / store & advance)"));
  Serial.println(F("  s  - print status"));
  Serial.println(F("  x  - clear all EEPROM cal data"));
  Serial.println(F("-- Mag calibration --"));
  Serial.println(F("  m  - reset & start mag calibration (rotate slowly 360)"));
  Serial.println(F("  d  - stop mag calibration & compute offsets"));
  Serial.println(F("-- Heading --"));
  Serial.println(F("  a  - stream heading angle (any key to stop)"));
  Serial.println(F("  t  - tare heading to zero"));
  Serial.println(F("-- Steering --"));
  Serial.println(F("  g      - start Kp steering toward target"));
  Serial.println(F("  p      - stop steering"));
  Serial.println(F("  <num>  - set new target heading (e.g. 90.5)"));
  Serial.println(F("  k      - set Kp gain (then type value + enter)"));
  Serial.println(F("-- Buttons --"));
  Serial.println(F("  GPIO 8 - tare heading"));
  Serial.println(F("  GPIO 9 - toggle steering on/off"));
  Serial.println(F("  h      - print this help"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("servo heading:  ")); Serial.println(heading);
  Serial.print(F("center:         ")); Serial.println(servo_center);
  Serial.print(F("left limit:     ")); Serial.println(servo_left_limit);
  Serial.print(F("right limit:    ")); Serial.println(servo_right_limit);
  Serial.print(F("servo valid:    ")); Serial.println(valid ? "yes" : "no");
  Serial.print(F("mag calibrated: ")); Serial.println(mag_calibrated ? "yes" : "no");
  Serial.print(F("steering:       ")); Serial.println(steering ? "on" : "off");
  Serial.print(F("target heading: ")); Serial.println(target_heading);
  Serial.print(F("Kp:             ")); Serial.println(Kp);
  Serial.print(F("tolerance:      ")); Serial.println(heading_tolerance);
  Serial.println(F("--------------"));
}

void clearAllCal() {
  sc.clearLocation(CENTER_LOCATION);
  sc.clearLocation(LEFT_LOCATION);
  sc.clearLocation(RIGHT_LOCATION);
  EEPROM.put(VALID_LOCATION,       false);
  EEPROM.put(MAG_VALID_LOCATION,   (uint8_t)0x00);
  EEPROM.put(STEER_VALID_LOCATION, (uint8_t)0x00);
  Serial.println(F("EEPROM cleared. Reset board to restart calibration."));
}

// ── Magnetometer calibration ──

void magStartCollecting() {
  // Clear any saved mag calibration from EEPROM
  EEPROM.put(MAG_VALID_LOCATION, (uint8_t)0x00);
  mag_calibrated = false;
  mag_collecting = true;
  mag_min_x =  32767;  mag_max_x = -32768;
  mag_min_y =  32767;  mag_max_y = -32768;
  Serial.println(F("Mag cal reset. Rotate sensor slowly through full 360."));
  Serial.println(F("Send 'd' when done."));
}

void magStopCollecting() {
  mag_collecting = false;

  range_x = mag_max_x - mag_min_x;
  range_y = mag_max_y - mag_min_y;

  if (range_x == 0 || range_y == 0) {
    Serial.println(F("Calibration failed: no variation detected. Try again."));
    return;
  }

  offset_x = (mag_max_x + mag_min_x) / 2.0;
  offset_y = (mag_max_y + mag_min_y) / 2.0;
  mag_calibrated = true;

  // Save to EEPROM
  EEPROM.put(MAG_OFFSET_X_LOCATION, offset_x);
  EEPROM.put(MAG_OFFSET_Y_LOCATION, offset_y);
  EEPROM.put(MAG_RANGE_X_LOCATION,  range_x);
  EEPROM.put(MAG_RANGE_Y_LOCATION,  range_y);
  EEPROM.put(MAG_VALID_LOCATION,    (uint8_t)0xB6);

  Serial.println(F("Mag calibration complete (saved to EEPROM):"));
  Serial.print(F("  offset_x = ")); Serial.println(offset_x);
  Serial.print(F("  offset_y = ")); Serial.println(offset_y);
  Serial.print(F("  range_x  = ")); Serial.println(range_x);
  Serial.print(F("  range_y  = ")); Serial.println(range_y);
}

void magUpdate() {
  mag.read();
  if (mag_collecting) {
    if (mag.m.x < mag_min_x) mag_min_x = mag.m.x;
    if (mag.m.x > mag_max_x) mag_max_x = mag.m.x;
    if (mag.m.y < mag_min_y) mag_min_y = mag.m.y;
    if (mag.m.y > mag_max_y) mag_max_y = mag.m.y;
  }
}

// Get calibrated X (-1.0 to 1.0)
float magCalibratedX() {
  return (mag.m.x - offset_x) / (range_x / 2.0);
}

// Get calibrated Y (-1.0 to 1.0)
float magCalibratedY() {
  return (mag.m.y - offset_y) / (range_y / 2.0);
}

// ── Heading angle ──

// Returns raw heading in degrees (-180 to +180)
float magHeadingRaw() {
  float cx = magCalibratedX();
  float cy = magCalibratedY();
  return atan2(cy, cx) * 180.0 / PI;
}

// Returns heading relative to tare point (-180 to +180)
float magHeading() {
  float h = magHeadingRaw() - tare_offset;
  // wrap to -180..+180
  if (h >  180.0) h -= 360.0;
  if (h < -180.0) h += 360.0;
  return h;
}

void tareHeading() {
  tare_offset = magHeadingRaw();
  Serial.println(F("Heading tared to 0."));
}

// ── Steering ──

void steerUpdate() {
  float current = magHeading();
  float error   = target_heading - current;

  Serial.print(current);
  Serial.print(F("  "));
  Serial.print(error);

  // wrap error to -180..+180
  if (error >  180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;

  // within tolerance — hold center
  if (fabs(error) <= heading_tolerance) {
    servo.writeMicroseconds(servo_center);
    return;
  }

  // Kp output: positive error → steer right, negative → steer left
  float output = Kp * error;

  // map output to servo microseconds from center
  int16_t servo_cmd;
  servo_cmd = servo_center - (int16_t)output;
  
  Serial.print(F("  "));
  Serial.print(servo_cmd);

  // clamp to calibrated servo limits
  if (servo_cmd < servo_left_limit && servo_left_limit < servo_center)
    servo_cmd = servo_left_limit;
  else if (servo_cmd < servo_center - 200) // safety fallback
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
  Wire.begin();

  if (!mag.init()) {
    Serial.println(F("Failed to detect LIS3MDL magnetometer!"));
    while (1);
  }
  mag.enableDefault();

  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  pinMode(BTN_L, INPUT_PULLUP);
  pinMode(BTN_R, INPUT_PULLUP);

  ledTimerInit_1ms();

  sc.enter();

  // Load mag calibration from EEPROM if previously saved
  uint8_t mag_magic;
  EEPROM.get(MAG_VALID_LOCATION, mag_magic);
  if (mag_magic == 0xB6) {
    EEPROM.get(MAG_OFFSET_X_LOCATION, offset_x);
    EEPROM.get(MAG_OFFSET_Y_LOCATION, offset_y);
    EEPROM.get(MAG_RANGE_X_LOCATION,  range_x);
    EEPROM.get(MAG_RANGE_Y_LOCATION,  range_y);
    mag_calibrated = true;
    Serial.println(F("Mag calibration loaded from EEPROM."));
  }

  // Load steering params from EEPROM if previously saved
  uint8_t steer_magic;
  EEPROM.get(STEER_VALID_LOCATION, steer_magic);
  if (steer_magic == 0xC4) {
    EEPROM.get(KP_LOCATION,      Kp);
    EEPROM.get(HEADING_LOCATION, target_heading);
    Serial.println(F("Steering params loaded from EEPROM."));
  }

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

  // GPIO 8 → tare
  if (btnL_pressed && mag_calibrated) tareHeading();

  // GPIO 9 → toggle steering
  if (btnR_pressed) {
    if (!steering) {
      if (!mag_calibrated) {
        Serial.println(F("Mag not calibrated."));
      } else if (!valid) {
        Serial.println(F("Servo not calibrated."));
      } else {
        steering = true;
        solPulsed = false;
        Serial.print(F("Steering ON. Target="));
        Serial.println(target_heading);
      }
    } else {
      steering = false;
      digitalWrite(SOLENOID_PIN, LOW);
      solPulsed = false;
      servo.writeMicroseconds(servo_center);
      Serial.println(F("Steering stopped."));
    }
  }

  magUpdate();

  // Live heading stream mode
  if (streaming) {
    if (mag_calibrated) {
      Serial.print(F("heading: "));
      Serial.println(magHeading(), 1);
    }
    delay(100); // 10 Hz update
    if (Serial.available()) {
      Serial.read(); // consume the stop key
      streaming = false;
      Serial.println(F("Streaming stopped."));
    }
    return;
  }

  // Kp steering loop
  if (steering) {
    // Solenoid firing (non-blocking)
    if (!solPulsed) {
      digitalWrite(SOLENOID_PIN, HIGH);
      solPulsed = true;
      solTimer = now;
    }
    if (now - solTimer >= PULSE_MS)        digitalWrite(SOLENOID_PIN, LOW);
    if (now - solTimer >= FIRE_INTERVAL_MS) solPulsed = false;

    steerUpdate();
    delay(20); // 50 Hz control loop
    // check for stop command
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p') {
        steering = false;
        digitalWrite(SOLENOID_PIN, LOW);
        solPulsed = false;
        servo.writeMicroseconds(servo_center);
        Serial.println(F("Steering stopped."));
      }
    }
    return;
  }

  if (!Serial.available()) return;

  char c = Serial.read();

  // Numeric input → set target heading
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
      EEPROM.put(KP_LOCATION,         Kp);
      EEPROM.put(STEER_VALID_LOCATION, (uint8_t)0xC4);
      Serial.print(F("Kp set to: "));
      Serial.println(Kp);
    } else {
      target_heading = val;
      EEPROM.put(HEADING_LOCATION,    target_heading);
      EEPROM.put(STEER_VALID_LOCATION, (uint8_t)0xC4);
      Serial.print(F("Target heading set to: "));
      Serial.println(target_heading);
    }
    return;
  }
  serialBufIdx = 0; // clear buffer on any command letter
  serialSettingKp = false;

  uint8_t button = 0;

  switch (c) {
    case 'l': button = 1; break;
    case 'r': button = 2; break;
    case 'b': button = 3; break;
    case 's': printStatus(); return;
    case 'x': clearAllCal(); return;
    case 'm': magStartCollecting(); return;
    case 'd': magStopCollecting();  return;
    case 'a': // stream heading
      if (!mag_calibrated) {
        Serial.println(F("Mag not calibrated. Run 'm' then 'd' first."));
      } else {
        streaming = true;
        Serial.println(F("Streaming heading. Send any key to stop."));
      }
      return;
    case 't': // tare
      if (!mag_calibrated) {
        Serial.println(F("Mag not calibrated. Run 'm' then 'd' first."));
      } else {
        tareHeading();
      }
      return;
    case 'g': // start steering
      if (!mag_calibrated) {
        Serial.println(F("Mag not calibrated first."));
        return;
      }
      if (!valid) {
        Serial.println(F("Servo not calibrated first."));
        return;
      }
      steering = true;
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
