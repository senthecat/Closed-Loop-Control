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
//  24       1    uint8_t  Mag heading FIFO size (1–20, default 10)
//
//  25       1    uint8_t  Steering valid magic (0xC4 = valid)
//  26       4    float    Kp gain
//  30       4    float    Target heading (degrees)
//
//  34       1    uint8_t  Trial count
//  35+     12    3xfloat  Trial rows: Kp, heading, MAE  (max 82 trials)
//
//  Total: 34 + 1 + 82*12 = 1019 bytes
// ─────────────────────────────────────────────────────────────────────────────

// Servo calibration  [0–6]
static const uint8_t VALID_LOCATION           = 0;
static const uint8_t CENTER_LOCATION          = 1;
static const uint8_t LEFT_LOCATION            = 3;
static const uint8_t RIGHT_LOCATION           = 5;

// Magnetometer calibration  [7–24]
static const uint8_t MAG_VALID_LOCATION       = 7;
static const uint8_t MAG_OFFSET_X_LOCATION    = 8;
static const uint8_t MAG_OFFSET_Y_LOCATION    = 12;
static const uint8_t MAG_RANGE_X_LOCATION     = 16;
static const uint8_t MAG_RANGE_Y_LOCATION     = 20;
static const uint8_t MAG_FIFO_SIZE_LOCATION   = 24;

// Steering parameters  [25–33]
static const uint8_t STEER_VALID_LOCATION     = 25;
static const uint8_t KP_LOCATION              = 26;
static const uint8_t HEADING_LOCATION         = 30;

// Trial log  [34–1018]
static const uint8_t  TRIAL_COUNT_LOCATION = 34;
static const uint8_t  TRIAL_DATA_START     = 35;
static const uint8_t  TRIAL_ROW_SIZE       = 12;  // 3 * sizeof(float): Kp, heading, MAE
static const uint8_t  MAX_TRIALS           = 82;  // (1024 - 35) / 12 = 82
