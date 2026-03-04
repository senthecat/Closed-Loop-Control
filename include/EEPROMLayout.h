#pragma once
#include <stdint.h>

// ── EEPROM Memory Map ─────────────────────────────────────────────────────────
//
//  Offset  Size  Type       Description
//  ──────  ────  ─────────  ─────────────────────────────────────────
//   0       1    uint8_t    Servo valid magic  (0xA5 = valid)
//   1       2    int16_t    Servo center µs
//   3       2    int16_t    Servo left-limit µs
//   5       2    int16_t    Servo right-limit µs
//
//   7       1    uint8_t    Mag valid magic    (0xA5 = valid)
//   8       6    int16_t[3] Mag min vector (x, y, z)
//  14       6    int16_t[3] Mag max vector (x, y, z)
//
//  20       1    uint8_t    Steering valid magic (0xC4 = valid)
//  21       4    float      Kp gain
//  25       4    float      Target heading (degrees)
//
//  Total: 29 bytes
// ─────────────────────────────────────────────────────────────────────────────

// Servo calibration  [0–6]
static const uint8_t VALID_LOCATION        = 0;
static const uint8_t CENTER_LOCATION       = 1;
static const uint8_t LEFT_LOCATION         = 3;
static const uint8_t RIGHT_LOCATION        = 5;

// Magnetometer calibration  [7–19]
static const uint8_t MAG_VALID_LOCATION    = 7;
static const uint8_t MAG_MIN_LOCATION      = 8;
static const uint8_t MAG_MAX_LOCATION      = 14;

// Steering parameters  [20–28]
static const uint8_t STEER_VALID_LOCATION  = 20;
static const uint8_t KP_LOCATION           = 21;
static const uint8_t HEADING_LOCATION      = 25;
