#include <Arduino.h>
#include <ServoControl.h>
#include <Interrupt.h>

Servo servo;
ServoControl sc;

// the pin numbers are temporary pls change later
const uint8_t leftButtonPin = 20;
const uint8_t rightButtonPin = 21;

volatile int currentOutput = 0;  // 0 = none, 1 = left, 2 = right, 3 = both

void setup() {
  Serial.begin(115200);
  ledTimerInit_1ms();

  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);

  sc = ServoControl(servo);
  sc.enter();

}

void loop() {
  // Read buttons (LOW = pressed because of pullup)
  bool leftPressed  = (digitalRead(leftButtonPin)  == LOW);
  bool rightPressed = (digitalRead(rightButtonPin) == LOW);

  int newOutput = 0;

  if (leftPressed && rightPressed) {
    newOutput = 3;
  }
  else if (leftPressed) {
    newOutput = 1;
  }
  else if (rightPressed) {
    newOutput = 2;
  }
  else {
    newOutput = 0;
  }

  // Only update when state changes (prevents spam)
  if (newOutput != currentOutput) {
    currentOutput = newOutput;
    Serial.println(currentOutput);
    sc.update(currentOutput);
  }
  
}
