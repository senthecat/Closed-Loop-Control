#include <Arduino.h>

#define LED_BUILTIN 10

enum LedMode : uint8_t { LED_OFF, LED_ON, LED_BLINK };

volatile LedMode   g_ledMode       = LED_OFF;
volatile uint16_t  g_blinkPeriodMs = 500;  // full toggle period
volatile uint16_t  g_msCounter      = 0;
volatile uint8_t   g_ledState       = 0;

// --- 1 ms tick using Timer2 compare match A ---
// (Timer1 is reserved by the Servo library on AVR)
ISR(TIMER2_COMPA_vect) {
  if (g_ledMode == LED_BLINK) {
    g_msCounter++;
    if (g_msCounter >= g_blinkPeriodMs) {
      g_msCounter = 0;
      g_ledState ^= 1;
      // Fast direct port write for D13 (PB5) instead of digitalWrite()
      if (g_ledState) PORTB |=  _BV(PB5);
      else           PORTB &= ~_BV(PB5);
    }
  }
}

// Call once in setup()
void ledTimerInit_1ms() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  cli();                  // disable interrupts globally
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  // 16 MHz / 128 = 125 kHz timer clock => 1 ms = 125 counts
  OCR2A = 124;            // count 0..124 = 125 ticks = 1 ms
  TCCR2A |= _BV(WGM21);  // CTC mode
  TCCR2B |= _BV(CS22) | _BV(CS20);  // prescaler 128
  TIMSK2 |= _BV(OCIE2A); // enable compare match A interrupt
  sei();                  // enable interrupts globally
}

// Public API (call from loop / FSM)
void ledSetOff() {
  cli();
  g_ledMode = LED_OFF;
  sei();
  digitalWrite(LED_BUILTIN, LOW);
}

void ledSetOn() {
  cli();
  g_ledMode = LED_ON;
  sei();
  digitalWrite(LED_BUILTIN, HIGH);
}

void ledSetBlink(uint16_t periodMs) {
  if (periodMs < 10) periodMs = 10; // sanity
  cli();
  g_blinkPeriodMs = periodMs;
  g_msCounter = 0;
  g_ledState = 0;
  g_ledMode = LED_BLINK;
  sei();
  digitalWrite(LED_BUILTIN, LOW);
}