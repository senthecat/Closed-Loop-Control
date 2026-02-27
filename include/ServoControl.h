#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Interrupt.h>

#define SERVO_PIN 3

const uint8_t VALID_LOCATION = 0;
const uint8_t CENTER_LOCATION = 1;
const uint8_t RIGHT_LOCATION = 2;
const uint8_t LEFT_LOCATION = 3;

uint16_t heading = 1500;

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

        EEPROM.get(VALID_LOCATION, valid);

        if (!valid) {   // invalid take measurements;
            ledSetBlink(250);
        } else {
            ledSetOn();
        }

        EEPROM.get(CENTER_LOCATION, servo_center);
        EEPROM.get(LEFT_LOCATION, servo_left_limit);
        EEPROM.get(RIGHT_LOCATION, servo_right_limit);

        if (servo_center == -1) servo_center = heading;  // change this later to (nominal_center);
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
                if (servo_center != -1) {   // valid stored
                    if (button == 1) 
                        clearLocation(CENTER_LOCATION);    // reset pressed
                    else
                        state = P2; // continue pressed next phase
                } else if (button == 1) {   // left
                    heading--;
                    servo.write(heading);
                } else if (button == 2) {   // right
                    heading++;
                    servo.write(heading);
                } else {   // both pressed -> store -> nextstate
                    EEPROM.put(CENTER_LOCATION, heading);
                    servo_center = heading;
                    state = P2;
                    ledSetBlink(50);    // blink rapidly
                    delay(500);
                    ledSetBlink(250);   // reset to normal blink rate
                }

                break;
            case P2:
                if (servo_left_limit != -1) {
                    if (button == 1) 
                        clearLocation(LEFT_LOCATION);
                    else
                        state = P3;
                } else if (button == 1) { // left 
                    heading -= 2;    // 2 because 1 is probably too slow can change later
                    servo.write(heading);
                } else if (button == 2) { // right
                    heading += 2;
                    servo.write(heading);
                } else {
                    EEPROM.put(LEFT_LOCATION, heading);
                    servo_left_limit = heading;
                    ledSetBlink(50); 
                    delay(500);
                    ledSetBlink(250);
                }
                
                break;
            case P3:
                if (servo_right_limit != -1) {
                    if (button == 1) 
                        clearLocation(RIGHT_LOCATION);
                    else
                        state = RUN;
                } else if (button == 1) { // left 
                    heading -= 2;    // 2 because 1 is probably too slow can change later
                    servo.write(heading);
                } else if (button == 2) { // right
                    heading += 2;
                    servo.write(heading);
                } else {
                    EEPROM.put(RIGHT_LOCATION, heading);
                    servo_right_limit = heading;
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
        EEPROM.put(idx, -1);
        valid = false;
    }

    void exit();
};