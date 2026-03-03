#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <ServoControl.h>

#define BTN_L   8
#define BTN_R   9

Servo servo;
ServoControl sc(servo);

LIS3MDL mag;

// Magnetometer calibration state
bool mag_collecting = false;
bool mag_calibrated = false;
int16_t mag_min_x, mag_max_x, mag_min_y, mag_max_y;
float offset_x, offset_y;
float range_x, range_y;

void printHelp() {
  Serial.println(F("=== Servo Calibration Test ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  l  - left  (button 1)"));
  Serial.println(F("  r  - right (button 2)"));
  Serial.println(F("  b  - both  (button 3 / store & advance)"));
  Serial.println(F("  s  - print status"));
  Serial.println(F("  x  - clear all EEPROM cal data"));
  Serial.println(F("  m  - start mag calibration (rotate sensor slowly)"));
  Serial.println(F("  d  - stop mag calibration & compute offsets"));
  Serial.println(F("  h  - print this help"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("heading:      ")); Serial.println(heading);
  Serial.print(F("center:       ")); Serial.println(servo_center);
  Serial.print(F("left limit:   ")); Serial.println(servo_left_limit);
  Serial.print(F("right limit:  ")); Serial.println(servo_right_limit);
  Serial.print(F("valid:        ")); Serial.println(valid ? "yes" : "no");
  Serial.println(F("--------------"));
}

void clearAllCal() {
  sc.clearLocation(CENTER_LOCATION);
  sc.clearLocation(LEFT_LOCATION);
  sc.clearLocation(RIGHT_LOCATION);
  EEPROM.put(VALID_LOCATION, false);
  Serial.println(F("EEPROM cleared. Reset board to restart calibration."));
}

void magStartCollecting() {
  mag_collecting = true;
  // Initialize min/max to opposite extremes so first reading overwrites them
  mag_min_x =  32767;  mag_max_x = -32768;
  mag_min_y =  32767;  mag_max_y = -32768;
  Serial.println(F("Mag calibration started. Rotate sensor slowly through full 360."));
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

  Serial.println(F("Mag calibration complete:"));
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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mag.init()) {
    Serial.println(F("Failed to detect LIS3MDL magnetometer!"));
    while (1);
  }
  mag.enableDefault();

  ledTimerInit_1ms();

  sc.enter();

  printHelp();
  printStatus();
}

void loop() {
  magUpdate();

  if (!Serial.available()) return;

  char c = Serial.read();
  uint8_t button = 0;

  switch (c) {
    case 'l': button = 1; break;
    case 'r': button = 2; break;
    case 'b': button = 3; break;
    case 's': printStatus(); return;
    case 'x': clearAllCal(); return;
    case 'm': magStartCollecting(); return;
    case 'd': magStopCollecting();  return;
    case 'h': printHelp();   return;
    case '\n':
    case '\r': return;  // ignore newlines
    default:
      Serial.print(F("Unknown command: "));
      Serial.println(c);
      return;
  }

  sc.update(button);
  printStatus();
}
