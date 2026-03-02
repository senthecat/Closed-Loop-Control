#include <Arduino.h>
#include <ServoControl.h>
#include <Interrupt.h>

Servo servo;
ServoControl sc(servo);

void printHelp() {
  Serial.println(F("=== Servo Calibration Test ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  l  - left  (button 1)"));
  Serial.println(F("  r  - right (button 2)"));
  Serial.println(F("  b  - both  (button 3 / store & advance)"));
  Serial.println(F("  s  - print status"));
  Serial.println(F("  x  - clear all EEPROM cal data"));
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

void setup() {
  Serial.begin(115200);
  ledTimerInit_1ms();

  sc.enter();

  printHelp();
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  uint8_t button = 0;

  switch (c) {
    case 'l': button = 1; break;
    case 'r': button = 2; break;
    case 'b': button = 3; break;
    case 's': printStatus(); return;
    case 'x': clearAllCal(); return;
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
