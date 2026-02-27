#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

#include "calibration\Calibrate.cpp"

// --------------- CONSTANTS -------------------
enum Mode {
  CALMAG,
  RUN
};

void setup() {
  Mode mode = CALMAG; // default to calibration mode
}

void loop() {
  case ()
}
