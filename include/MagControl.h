#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <EEPROMLayout.h>

struct MagControl {
private:
    LIS3MDL _mag;

    // Calibration data (stored in EEPROM)
    float _offset_x, _offset_y;
    float _range_x,  _range_y;

    // Running min/max used only during active calibration
    int16_t _min_x, _max_x;
    int16_t _min_y, _max_y;

    bool  _calibrated;
    bool  _collecting;
    float _tare_offset;

    float _calibratedX() {
        return (_mag.m.x - _offset_x) / (_range_x / 2.0f);
    }

    float _calibratedY() {
        return (_mag.m.y - _offset_y) / (_range_y / 2.0f);
    }

public:
    MagControl() : _calibrated(false), _collecting(false), _tare_offset(0.0f) {}

    /**
     * Initialises I2C and the magnetometer, then loads calibration from EEPROM
     * if a valid magic byte is present. Call once from setup().
     */
    void enter() {
        Wire.begin();
        delay(10);

        if (!_mag.init()) {
            Serial.println(F("Failed to detect magnetometer."));
            while (1);
        }
        _mag.enableDefault();

        uint8_t magic;
        EEPROM.get(MAG_VALID_LOCATION, magic);
        if (magic == 0xB6) {
            EEPROM.get(MAG_OFFSET_X_LOCATION, _offset_x);
            EEPROM.get(MAG_OFFSET_Y_LOCATION, _offset_y);
            EEPROM.get(MAG_RANGE_X_LOCATION,  _range_x);
            EEPROM.get(MAG_RANGE_Y_LOCATION,  _range_y);
            _calibrated = true;
            Serial.println(F("Mag calibration loaded from EEPROM."));
        } else {
            Serial.println(F("No mag calibration found. Run 'm' to start."));
        }
    }

    /**
     * Reads the magnetometer and, if a calibration is in progress, updates
     * the running min/max. Call every loop iteration.
     */
    void update() {
        _mag.read();
        if (!_collecting) return;

        if (_mag.m.x < _min_x) _min_x = _mag.m.x;
        if (_mag.m.x > _max_x) _max_x = _mag.m.x;
        if (_mag.m.y < _min_y) _min_y = _mag.m.y;
        if (_mag.m.y > _max_y) _max_y = _mag.m.y;
    }

    /**
     * Resets running min/max and starts collecting calibration samples.
     * Triggered by serial command 'm'.
     */
    void startCal() {
        _min_x =  32767;  _max_x = -32768;
        _min_y =  32767;  _max_y = -32768;
        _collecting = true;
        Serial.println(F("Mag cal started. Rotate sensor slowly through 360 deg."));
        Serial.println(F("Send 'd' when done."));
    }

    /**
     * Stops collection, computes offset/range, and saves to EEPROM.
     * Triggered by serial command 'd'.
     */
    void finishCal() {
        _collecting = false;

        float range_x = _max_x - _min_x;
        float range_y = _max_y - _min_y;

        if (range_x == 0 || range_y == 0) {
            Serial.println(F("Cal failed: no variation detected. Try again."));
            return;
        }

        _offset_x = (_max_x + _min_x) / 2.0f;
        _offset_y = (_max_y + _min_y) / 2.0f;
        _range_x  = range_x;
        _range_y  = range_y;

        EEPROM.put(MAG_OFFSET_X_LOCATION, _offset_x);
        EEPROM.put(MAG_OFFSET_Y_LOCATION, _offset_y);
        EEPROM.put(MAG_RANGE_X_LOCATION,  _range_x);
        EEPROM.put(MAG_RANGE_Y_LOCATION,  _range_y);
        EEPROM.put(MAG_VALID_LOCATION,    (uint8_t)0xB6);
        _calibrated = true;

        Serial.println(F("Mag calibration saved:"));
        Serial.print(F("  offset_x = ")); Serial.println(_offset_x);
        Serial.print(F("  offset_y = ")); Serial.println(_offset_y);
        Serial.print(F("  range_x  = ")); Serial.println(_range_x);
        Serial.print(F("  range_y  = ")); Serial.println(_range_y);
    }

    /**
     * Clears magnetometer calibration from EEPROM.
     */
    void clearCal() {
        EEPROM.put(MAG_OFFSET_X_LOCATION, 0.0f);
        EEPROM.put(MAG_OFFSET_Y_LOCATION, 0.0f);
        EEPROM.put(MAG_RANGE_X_LOCATION,  0.0f);
        EEPROM.put(MAG_RANGE_Y_LOCATION,  0.0f);
        EEPROM.put(MAG_VALID_LOCATION,    (uint8_t)0x00);
        _calibrated = false;
    }

    bool isCalibrated() const { return _calibrated; }
    bool isCollecting()  const { return _collecting; }

    /** Returns raw heading in degrees (-180 to +180). */
    float headingRaw() {
        return atan2(_calibratedY(), _calibratedX()) * 180.0f / PI;
    }

    /** Returns heading relative to the tare point (-180 to +180). */
    float heading() {
        float h = headingRaw() - _tare_offset;
        if (h >  180.0f) h -= 360.0f;
        if (h < -180.0f) h += 360.0f;
        return h;
    }

    /** Sets the current heading as the zero reference. */
    void tare() {
        _tare_offset = headingRaw();
        Serial.println(F("Heading tared to 0."));
    }
};
