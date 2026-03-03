#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <Interrupt.h>
#include <LIS3MDL.h>
#include <Wire.h>

// EEPROM layout (follows servo cal data which ends at byte 6)
// byte 7:      MAG_VALID_LOCATION  (uint8_t magic)
// bytes 8-13:  MAG_MIN_LOCATION    (LIS3MDL::vector<int16_t>, 3 × int16_t = 6 bytes)
// bytes 14-19: MAG_MAX_LOCATION    (LIS3MDL::vector<int16_t>, 3 × int16_t = 6 bytes)
const uint8_t MAG_VALID_LOCATION = 7;
const uint8_t MAG_MIN_LOCATION   = MAG_VALID_LOCATION + sizeof(uint8_t);
const uint8_t MAG_MAX_LOCATION   = MAG_MIN_LOCATION   + sizeof(LIS3MDL::vector<int16_t>);

LIS3MDL mag;
LIS3MDL::vector<int16_t> mag_min;
LIS3MDL::vector<int16_t> mag_max;
boolean mag_valid;

struct MagControl {
    private:
    enum State { IDLE, CAL, DONE } state;
    LIS3MDL::vector<int16_t> running_min;
    LIS3MDL::vector<int16_t> running_max;
    uint32_t last_change_ms;
    bool stable;
    bool first_sample;

    static const uint32_t STABLE_MS = 2000UL;

    void _startCal() {
        running_min  = { 32767,  32767,  32767};
        running_max  = {-32768, -32768, -32768};
        stable       = false;
        first_sample = false;
        ledSetBlink(100);   // fast blink: no valid cal, rotate sensor
        state = CAL;
    }

    public:
    MagControl() : state{IDLE} {}

    void enter() {
        Wire.begin();
        delay(10);  // allow I2C bus to settle after init

        if (!mag.init()) {
            Serial.println(F("Failed to detect and initialize magnetometer."));
            while (1);
        }
        mag.enableDefault();

        uint8_t magic;
        EEPROM.get(MAG_VALID_LOCATION, magic);
        mag_valid = (magic == 0xA5);

        if (mag_valid) {
            EEPROM.get(MAG_MIN_LOCATION, mag_min);
            EEPROM.get(MAG_MAX_LOCATION, mag_max);
            ledSetBlink(500);   // slow blink: valid stored, waiting for input
            state = IDLE;
        } else {
            _startCal();
        }
    }

    /**
     * Call every loop iteration.
     *
     * IDLE state (valid stored):
     *   button 2 (right)  → accept stored values, advance to DONE
     *   button 1 (left)   → clear EEPROM, re-enter CAL
     *
     * CAL state:
     *   Reads mag each call, tracks running min/max across all 3 axes.
     *   LED blinks (100 ms) while any axis extreme is still changing.
     *   LED goes solid once no new extreme for STABLE_MS (2 s).
     *   button 3 (both)   → save to EEPROM (only accepted when stable)
     */
    void update(uint8_t button) {
        switch (state) {
            case IDLE:
                if (button == 0) break;

                if (button == 2) {          // right = continue
                    state = DONE;
                } else if (button == 1) {   // left = reset, re-calibrate
                    clearMag();
                    _startCal();
                }
                break;

            case CAL: {
                mag.read();

                bool changed = false;
                if (mag.m.x < running_min.x) { running_min.x = mag.m.x; changed = true; }
                if (mag.m.y < running_min.y) { running_min.y = mag.m.y; changed = true; }
                if (mag.m.z < running_min.z) { running_min.z = mag.m.z; changed = true; }
                if (mag.m.x > running_max.x) { running_max.x = mag.m.x; changed = true; }
                if (mag.m.y > running_max.y) { running_max.y = mag.m.y; changed = true; }
                if (mag.m.z > running_max.z) { running_max.z = mag.m.z; changed = true; }

                if (changed) {
                    if (!first_sample) first_sample = true;
                    last_change_ms = millis();
                    if (stable) {
                        // new extreme appeared after going stable — keep rotating
                        stable = false;
                        ledSetBlink(100);
                    }
                } else if (first_sample && !stable
                           && (millis() - last_change_ms >= STABLE_MS)) {
                    // no new extreme for 2 s → ready to save
                    stable = true;
                    ledSetOn();
                }

                // only accept save once stable (LED solid)
                if (button == 3 && stable) {
                    mag_min = running_min;
                    mag_max = running_max;
                    EEPROM.put(MAG_MIN_LOCATION,   mag_min);
                    EEPROM.put(MAG_MAX_LOCATION,   mag_max);
                    EEPROM.put(MAG_VALID_LOCATION, (uint8_t)0xA5);
                    mag_valid = true;
                    state = DONE;

                    ledSetBlink(50);    // rapid confirmation flash
                    delay(500);
                    ledSetOn();
                }
                break;
            }

            case DONE:
                break;
        }
    }

    bool isDone() const { return state == DONE; }

    void clearMag() {
        LIS3MDL::vector<int16_t> zero = {0, 0, 0};
        EEPROM.put(MAG_MIN_LOCATION,   zero);
        EEPROM.put(MAG_MAX_LOCATION,   zero);
        EEPROM.put(MAG_VALID_LOCATION, (uint8_t)0x00);
        mag_valid = false;
    }
};
