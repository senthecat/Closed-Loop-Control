#include <Arduino.h>

enum LedMode : uint8_t { LED_OFF, LED_ON, LED_BLINK };

volatile LedMode   g_ledMode       = LED_OFF;
volatile uint16_t  g_blinkPeriodMs = 500;  // full toggle period
volatile uint16_t  g_msCounter      = 0;
volatile uint8_t   g_ledState       = 0;

// --- 1 ms tick using Timer1 compare match A ---
ISR(TIMER1_COMPA_vect) {
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

  cli();                 // disable interrupts globally
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // 16 MHz / 64 = 250 kHz timer clock => 1 ms = 250 counts
  OCR1A = 249;           // count 0..249 = 250 ticks = 1 ms
  TCCR1B |= _BV(WGM12);  // CTC mode
  TCCR1B |= _BV(CS11) | _BV(CS10);  // prescaler 64
  TIMSK1 |= _BV(OCIE1A); // enable compare match A interrupt
  sei();                 // enable interrupts globally
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