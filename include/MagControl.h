#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <EEPROMLayout.h>

struct MagControl {
private:
    LIS3MDL _mag;
    LIS3MDL::vector<int16_t> _mag_min;
    LIS3MDL::vector<int16_t> _mag_max;
    LIS3MDL::vector<int16_t> _running_min;
    LIS3MDL::vector<int16_t> _running_max;
    bool  _calibrated;
    bool  _collecting;
    float _tare_offset;

    float _calibratedX() {
        float range = _mag_max.x - _mag_min.x;
        if (range == 0) return 0;
        return (_mag.m.x - (_mag_max.x + _mag_min.x) / 2.0f) / (range / 2.0f);
    }

    float _calibratedY() {
        float range = _mag_max.y - _mag_min.y;
        if (range == 0) return 0;
        return (_mag.m.y - (_mag_max.y + _mag_min.y) / 2.0f) / (range / 2.0f);
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
        if (magic == 0xA5) {
            EEPROM.get(MAG_MIN_LOCATION, _mag_min);
            EEPROM.get(MAG_MAX_LOCATION, _mag_max);
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

        if (_mag.m.x < _running_min.x) _running_min.x = _mag.m.x;
        if (_mag.m.y < _running_min.y) _running_min.y = _mag.m.y;
        if (_mag.m.z < _running_min.z) _running_min.z = _mag.m.z;
        if (_mag.m.x > _running_max.x) _running_max.x = _mag.m.x;
        if (_mag.m.y > _running_max.y) _running_max.y = _mag.m.y;
        if (_mag.m.z > _running_max.z) _running_max.z = _mag.m.z;
    }

    /**
     * Resets running min/max and starts collecting calibration samples.
     * Triggered by serial command 'm'.
     */
    void startCal() {
        _running_min = { 32767,  32767,  32767};
        _running_max = {-32768, -32768, -32768};
        _collecting  = true;
        Serial.println(F("Mag cal started. Rotate sensor slowly through 360 deg."));
        Serial.println(F("Send 'd' when done."));
    }

    /**
     * Stops collection, validates the data, and saves to EEPROM.
     * Triggered by serial command 'd'.
     */
    void finishCal() {
        _collecting = false;

        if (_running_max.x == _running_min.x || _running_max.y == _running_min.y) {
            Serial.println(F("Cal failed: no variation detected. Try again."));
            return;
        }

        _mag_min = _running_min;
        _mag_max = _running_max;
        EEPROM.put(MAG_MIN_LOCATION,   _mag_min);
        EEPROM.put(MAG_MAX_LOCATION,   _mag_max);
        EEPROM.put(MAG_VALID_LOCATION, (uint8_t)0xA5);
        _calibrated = true;

        Serial.println(F("Mag calibration saved."));
        Serial.print(F("  min: x=")); Serial.print(_mag_min.x);
        Serial.print(F(" y="));       Serial.println(_mag_min.y);
        Serial.print(F("  max: x=")); Serial.print(_mag_max.x);
        Serial.print(F(" y="));       Serial.println(_mag_max.y);
    }

    /**
     * Clears magnetometer calibration from EEPROM.
     */
    void clearCal() {
        LIS3MDL::vector<int16_t> zero = {0, 0, 0};
        EEPROM.put(MAG_MIN_LOCATION,   zero);
        EEPROM.put(MAG_MAX_LOCATION,   zero);
        EEPROM.put(MAG_VALID_LOCATION, (uint8_t)0x00);
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
