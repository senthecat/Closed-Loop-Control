#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Interrupt.h>

#define SERVO_PIN 3


// probably move these all into one file for easier readability and less confusion
const uint8_t VALID_LOCATION = 0;
const uint8_t CENTER_LOCATION = 1 * sizeof(uint16_t);
const uint8_t RIGHT_LOCATION = 2 * sizeof(uint16_t);
const uint8_t LEFT_LOCATION = 3 * sizeof(uint16_t);

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
        EEPROM.get(VALID_LOCATION, valid);

        if (!valid) {   // invalid take measurements;
            ledSetBlink(250);
        } else {
            ledSetOn();
        }

        EEPROM.get(CENTER_LOCATION, servo_center);
        EEPROM.get(LEFT_LOCATION, servo_left_limit);
        EEPROM.get(RIGHT_LOCATION, servo_right_limit);

        if (servo_center == 0) servo_center = heading; // servo_center empty
    };
    
    /**
     * Proceeds through three phases.
     * Phase 1:
     *  Center Location calibration. 
     *  Reset button press, clears the EEPROM.
     *  If (valid->center) proceed to phase 2.
     * Phase 2:
     *  Right Location calibration.
     *  Reset button press, clears the EEPROM.
     *  If (valid->right) proceed to phase 3.
     * Phase 3:
     *  Left Location calibration.
     *  Reset button press, clearsr the EEPROM.
     *  If (valid ->left) proceed to next state.
     * 
     * @return boolean
     */
    boolean update(uint8_t button) {
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
                    heading--;
                    servo.writeMicroseconds(heading);
                } else if (button == 2) {   // right
                    heading++;
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
                        heading = servo_center;
                    }
                } else if (button == 1) { // left 
                    heading -= 2;    // 2 because 1 is probably too slow can change later
                    servo.writeMicroseconds(heading);
                } else if (button == 2) { // right
                    heading += 2;
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
                        heading = servo_center;
                    }
                } else if (button == 1) { // left 
                    heading -= 2;    // 2 because 1 is probably too slow can change later
                    servo.writeMicroseconds(heading);
                } else if (button == 2) { // right
                    heading += 2;
                    servo.writeMicroseconds(heading);
                } else if (button == 3) {
                    EEPROM.put(RIGHT_LOCATION, heading);
                    EEPROM.put(VALID_LOCATION, true);
                    valid = true;

                    servo_right_limit = heading;
                    state = RUN;

                    ledSetBlink(50); 
                    delay(500);
                    ledSetBlink(250);
                }
                
                break;
            case RUN:   // does nothing right now
                break;
        }
    }

    /**
     * Clears position at specific EEPROM location
     */
    boolean clearLocation(uint8_t idx) {
        EEPROM.put(idx, 0); // should be fine to write 0 since we are measuring ms and this should write false to valid
        valid = false;
    }

    void exit();
};