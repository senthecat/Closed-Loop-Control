#pragma once
#include <Arduino.h>
#include <math.h>

// Inches traveled per limit switch trigger
#define NAV_DIST_PER_TICK  1.16f
// Arrival radius in inches (1 circular foot)
#define NAV_ARRIVAL_IN    12.0f

// ── WayPoint ──────────────────────────────────────────────────────────────────
// X/Y position in inches relative to the robot's starting location.
// Coordinate frame matches the magnetometer: heading 0° = +X direction.
// ─────────────────────────────────────────────────────────────────────────────
struct WayPoint {
    float x;
    float y;
};

// ── NavSystem ─────────────────────────────────────────────────────────────────
// Dead-reckoning navigator. Position is updated once per limit switch trigger.
// The caller is responsible for setting target_heading to bearingToNext()
// after each update.
// ─────────────────────────────────────────────────────────────────────────────
struct NavSystem {
private:
    const WayPoint* _wps;
    uint8_t         _count;
    uint8_t         _idx;     // index of current target waypoint
    float           _x;       // estimated X position (inches)
    float           _y;       // estimated Y position (inches)

public:
    NavSystem() : _wps(nullptr), _count(0), _idx(0), _x(0.0f), _y(0.0f) {}

    // Call once from setup() with your waypoint array and element count.
    void begin(const WayPoint* wps, uint8_t count) {
        _wps   = wps;
        _count = count;
        reset();
    }

    // Reset position to origin and restart from waypoint 0.
    void reset() {
        _idx = 0;
        _x   = 0.0f;
        _y   = 0.0f;
    }

    // Call once per limit switch trigger with the current heading (degrees,
    // tared). Updates dead-reckoning position by NAV_DIST_PER_TICK inches,
    // auto-advances to the next waypoint if within NAV_ARRIVAL_IN, and
    // returns the new target bearing toward the current waypoint.
    float update(float heading_deg) {
        float rad = heading_deg * (PI / 180.0f);
        _x += NAV_DIST_PER_TICK * cosf(rad);
        _y += NAV_DIST_PER_TICK * sinf(rad);

        if (!done() && distToNext() <= NAV_ARRIVAL_IN) {
            Serial.print(F("Nav: waypoint "));
            Serial.print(_idx);
            Serial.println(F(" reached."));
            _idx++;
            if (done()) {
                Serial.println(F("Nav: all waypoints complete."));
            } else {
                Serial.print(F("Nav: heading to waypoint "));
                Serial.println(_idx);
            }
        }

        return bearingToNext(heading_deg);
    }

    // Bearing from current position to current target waypoint (degrees,
    // -180 to +180). Returns current_heading if all waypoints are done.
    float bearingToNext(float current_heading) const {
        if (done()) return current_heading;
        float dx = _wps[_idx].x - _x;
        float dy = _wps[_idx].y - _y;
        return atan2f(dy, dx) * (180.0f / PI);
    }

    // Straight-line distance to the current target waypoint (inches).
    float distToNext() const {
        if (done()) return 0.0f;
        float dx = _wps[_idx].x - _x;
        float dy = _wps[_idx].y - _y;
        return sqrtf(dx * dx + dy * dy);
    }

    bool    done()  const { return _idx >= _count; }
    uint8_t index() const { return _idx; }
    uint8_t count() const { return _count; }
    float   posX()  const { return _x; }
    float   posY()  const { return _y; }
};
