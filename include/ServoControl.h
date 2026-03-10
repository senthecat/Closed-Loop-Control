#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Interrupt.h>
#include <EEPROMLayout.h>

#define SERVO_PIN 3

uint16_t heading = 1500; // default as nominal center which should be 1500

int16_t servo_center;
int16_t servo_right_limit;
int16_t servo_left_limit;
boolean valid;

struct ServoControl{
    private:
    Servo servo;
    enum State {
        P1, // phase 1
        P2, // phase 2
        P3,  // phase 3
        RUN
    } state;

    public: 
    ServoControl(Servo s) : servo{s}, state{P1} {};

    
    /**
     * This that is always called to initialize the servo.
     */
    void enter() {
        servo.attach(SERVO_PIN);

        // this is used to check if we need to force calibration
        uint8_t magic;
        EEPROM.get(VALID_LOCATION, magic);
        valid = (magic == 0xA5);

        if (!valid) {   // invalid take measurements;
            ledSetBlink(250);   // fast blink, need to measure
        } else {
            ledSetBlink(500);   // slow blink valid, but need to confirm
        }

        EEPROM.get(CENTER_LOCATION, servo_center);
        EEPROM.get(LEFT_LOCATION, servo_left_limit);
        EEPROM.get(RIGHT_LOCATION, servo_right_limit);

        if (servo_center < 1600 && servo_center > 1400) {   // built in lower and upper to prevent damge
            heading = servo_center;
        }   // else leave heading default 

        // normalize fresh EEPROM (0xFF bytes read as -1) to sentinel 0 = "not set"
        if (servo_center == -1)      servo_center = 0;
        if (servo_left_limit == -1)  servo_left_limit = 0;
        if (servo_right_limit == -1) servo_right_limit = 0;
    };
    
    /**
     * Proceeds through three phases.
     * All phases: l=adjust left, r=adjust right, b=confirm & advance.
     * Use 'x' serial command to restart servo calibration from phase 1.
     */
    void update(uint8_t button) {
        switch (state) {
            case P1:
                if (button == 0) break;

                if (button == 1) {
                    heading -= 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 2) {
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(CENTER_LOCATION, heading);
                    servo_center = heading;
                    state = P2;
                    Serial.println(F("Center saved. Now set LEFT limit."));
                    ledSetBlink(50);
                    delay(500);
                    ledSetBlink(250);
                }
                break;

            case P2:
                if (button == 0) break;

                if (button == 1) {
                    heading -= 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 2) {
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(LEFT_LOCATION, heading);
                    servo_left_limit = heading;
                    heading = servo_center;
                    servo.writeMicroseconds(heading);
                    state = P3;
                    Serial.println(F("Left saved. Now set RIGHT limit."));
                    ledSetBlink(50);
                    delay(500);
                    ledSetBlink(250);
                }
                break;

            case P3:
                if (button == 0) break;

                if (button == 1) {
                    heading -= 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 2) {
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(RIGHT_LOCATION, heading);
                    EEPROM.put(VALID_LOCATION, (uint8_t)0xA5);
                    valid = true;
                    servo_right_limit = heading;
                    state = RUN;
                    Serial.println(F("Right saved. Servo calibration complete."));
                    ledSetBlink(50);
                    delay(500);
                    ledSetOn();
                }
                break;

            case RUN:
                break;
        }
    }

    /**
     * Resets servo calibration only — clears servo EEPROM locations and
     * returns to phase 1. Does not affect mag calibration.
     */
    void reset() {
        clearLocation(CENTER_LOCATION);
        clearLocation(LEFT_LOCATION);
        clearLocation(RIGHT_LOCATION);
        servo_center      = 0;
        servo_left_limit  = 0;
        servo_right_limit = 0;
        heading = 1500;
        servo.writeMicroseconds(heading);
        state = P1;
        ledSetBlink(250);
        Serial.println(F("Servo cal reset. Starting phase 1: set CENTER."));
    }

    /**
     * Clears position at specific EEPROM location
     */
    void clearLocation(uint8_t idx) {
        EEPROM.put(idx, (int16_t)0);
        EEPROM.put(VALID_LOCATION, (uint8_t)0x00);
        valid = false;
    }

    void exit();
};