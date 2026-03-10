#pragma once
#include <stdint.h>

// ── EEPROM Memory Map ─────────────────────────────────────────────────────────
//
//  Offset  Size  Type     Description
//  ──────  ────  ───────  ─────────────────────────────────────────
//   0       1    uint8_t  Servo valid magic  (0xA5 = valid)
//   1       2    int16_t  Servo center µs
//   3       2    int16_t  Servo left-limit µs
//   5       2    int16_t  Servo right-limit µs
//
//   7       1    uint8_t  Mag valid magic    (0xB6 = valid)
//   8       4    float    Mag offset X
//  12       4    float    Mag offset Y
//  16       4    float    Mag range X
//  20       4    float    Mag range Y
//
//  24       1    uint8_t  Steering valid magic (0xC4 = valid)
//  25       4    float    Kp gain
//  29       4    float    Target heading (degrees)
//
//  Total: 33 bytes
// ─────────────────────────────────────────────────────────────────────────────

// Servo calibration  [0–6]
static const uint8_t VALID_LOCATION        = 0;
static const uint8_t CENTER_LOCATION       = 1;
static const uint8_t LEFT_LOCATION         = 3;
static const uint8_t RIGHT_LOCATION        = 5;

// Magnetometer calibration  [7–23]
static const uint8_t MAG_VALID_LOCATION    = 7;
static const uint8_t MAG_OFFSET_X_LOCATION = 8;
static const uint8_t MAG_OFFSET_Y_LOCATION = 12;
static const uint8_t MAG_RANGE_X_LOCATION  = 16;
static const uint8_t MAG_RANGE_Y_LOCATION  = 20;

// Steering parameters  [24–32]
static const uint8_t STEER_VALID_LOCATION  = 24;
static const uint8_t KP_LOCATION           = 25;
static const uint8_t HEADING_LOCATION      = 29;
