#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Interrupt.h>

#define SERVO_PIN 3


// probably move these all into one file for easier readability and less confusion
const uint8_t VALID_LOCATION  = 0;
const uint8_t CENTER_LOCATION = VALID_LOCATION  + sizeof(uint8_t);
const uint8_t LEFT_LOCATION   = CENTER_LOCATION + sizeof(int16_t);
const uint8_t RIGHT_LOCATION  = LEFT_LOCATION   + sizeof(int16_t);

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
            ledSetBlink(250);
        } else {
            ledSetOn();
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
     * Phase 1:
     *  Center Location calibration. 
     *  Reset button press, clears the EEPROM.
     *  If (valid->center) proceed to phase 2.
     * Phase 2:
     *  Left Location calibration.
     *  Reset button press, clears the EEPROM.
     *  If (valid->left) proceed to phase 3.
     * Phase 3:
     *  Right Location calibration.
     *  Reset button press, clears the EEPROM.
     *  If (valid->right) proceed to next state.
     */
    void update(uint8_t button) {
        switch (state) {
            case P1:
                if (button == 0) break; // fast break dont compute unnecesary things

                if (servo_center != 0) {   // valid stored
                    if (button == 1) {
                        clearLocation(CENTER_LOCATION);    // reset pressed
                        servo.writeMicroseconds(heading);
                        servo_center = 0;
                    } else if (button == 2) {
                        state = P2; // continue pressed next phase
                        heading = servo_center;
                    }
                } else if (button == 1) {   // left
                    heading -= 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 2) {   // right
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {   // both pressed -> store -> nextstate
                    EEPROM.put(CENTER_LOCATION, heading);

                    servo_center = heading;
                    state = P2;

                    ledSetBlink(50);    // blink rapidly
                    delay(500);
                    ledSetBlink(250);   // reset to normal blink rate
                }

                break;
            case P2:
                if (button == 0) break;

                if (servo_left_limit != 0) {
                    if (button == 1) {
                        clearLocation(LEFT_LOCATION);
                        servo.writeMicroseconds(servo_center);  // move to center
                        servo_left_limit = 0;
                    } else if (button == 2) {
                        state = P3;
                    }
                    heading = servo_center;
                } else if (button == 1) { // left 
                    heading -= 5;    // 2 because 1 is probably too slow can change later
                    servo.writeMicroseconds(heading);
                } else if (button == 2) { // right
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(LEFT_LOCATION, heading);

                    servo_left_limit = heading;
                    state = P3;

                    ledSetBlink(50); 
                    delay(500);
                    ledSetBlink(250);
                }
                
                break;
            case P3:
                if (button == 0) break;

                if (servo_right_limit != 0) {
                    if (button == 1) {
                        clearLocation(RIGHT_LOCATION);
                        servo.writeMicroseconds(servo_center);  // move to center
                        servo_right_limit = 0;
                    } else if (button == 2) {
                        state = RUN;
                    }
                    heading = servo_center;
                } else if (button == 1) { // left 
                    heading -= 5;    // 2 because 1 is probably too slow can change later
                    servo.writeMicroseconds(heading);
                } else if (button == 2) { // right
                    heading += 5;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(RIGHT_LOCATION, heading);
                    EEPROM.put(VALID_LOCATION, (uint8_t)0xA5);
                    valid = true;

                    servo_right_limit = heading;
                    state = RUN;

                    ledSetBlink(50);
                    delay(500);
                    ledSetOn();
                }
                
                break;
            case RUN:   // does nothing right now
                servo.writeMicroseconds(servo_center);
                break;
        }
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