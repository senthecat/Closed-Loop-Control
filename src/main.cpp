#include <Arduino.h>

// note: entire FSM can be moved to external header for neatness, keep it mind limited storage though so condensing may be required

// system state enum
enum State {
  BOOT,
  CALIB_SERVO,
  CALIB_MAG,
  WAIT_START,
  RUN_NAV,
  FAULT
};

// event type classification
enum EventType {
  EVT_NONE,

  EVT_BTN_LEFT_DOWN,
  EVT_BTN_LEFT_UP,
  EVT_BTN_RIGHT_DOWN,
  EVT_BTN_RIGHT_UP,
  EVT_BTN_BOTH_DOWN,
  EVT_BTN_BOTH_UP,

  EVT_BTN_SHORT,        // optional convenience
  EVT_BTN_LONG,

  EVT_MAG_OK,
  EVT_MAG_BAD,

  EVT_TIMEOUT,
  EVT_ABORT,
  EVT_WAYPOINT_REACHED
};

// event data type
struct Event {
  EventType type;
  uint32_t t_ms;
  int32_t data;
};


/// Event queue handling
const uint8_t EVQ_CAP = 16;
Event evq[EVQ_CAP];
volatile int evq_head = 0;
volatile int evq_tail = 0;
volatile int next;

bool pushEvent(EventType type, int32_t data = 0) {
  next = (evq_head + 1) % EVQ_CAP;
  if (next == evq_tail) return false;  // queue is full
  evq[evq_head] = {type, millis(), data};
  evq_head = next;
  return true;
}

bool popEvent(Event &out) {
  if (evq_tail == evq_head) return false; // empty
  out = evq[evq_tail];
  evq_tail = (evq_tail + 1) % EVQ_CAP;
  return true;
}


/// Button Handling
#define BTN_L 0
#define BTN_R 1
bool leftPrev = false;
bool rightPrev = true;
volatile bool left;
volatile bool right;

void readInputs() {
  left = (digitalRead(BTN_L) == LOW);
  right = (digitalRead(BTN_R) == LOW);

  if (left != leftPrev) {
    pushEvent(left ? EVT_BTN_LEFT_DOWN : EVT_BTN_LEFT_UP);
    leftPrev = left;
  }
  if (right != rightPrev) {
    pushEvent(right ? EVT_BTN_RIGHT_DOWN : EVT_BTN_RIGHT_UP);
    rightPrev = right;
  }

  // both-down / both-up convenience
  bool both = left && right;
  static bool bothPrev = false;
  if (both != bothPrev) {
    pushEvent(both ? EVT_BTN_BOTH_DOWN : EVT_BTN_BOTH_UP);
    bothPrev = both;
  }
}


/// FSM
State g_state = BOOT;
void setState(State s) {
  onExit(g_state);
  g_state = s;
  onEntry(g_state);
}

void onEntry(State s) {
  switch (s) {
    case BOOT:
    break;
    
    case CALIB_SERVO:
    break;
    
    case CALIB_MAG:
    break;

    case WAIT_START:
    break;

    case RUN_NAV:
    break;

    case FAULT:
    break;
  }
}

void onExit(State s) {
  switch (s) {
    case CALIB_SERVO:
    break; // saving to EEPROM but might handle in class
    case CALIB_MAG:
    break;  // saving to EEPROM but might handle in class
    default:
    break;
  };
}

void update_BOOT(Event e) {
  // if servo state invalid setState(CALIB_SERVO); return;
  // if mag state invalid setState(CALIB_MAG); return;
  setState(WAIT_START);
}

void update_WAIT_START(Event e) {
  if (e.type == EVT_BTN_BOTH_DOWN) {
    setState(RUN_NAV);
    return;
  }

  // maybe add option to re-enter calibration state
}

void update_RUN_NAV(Event e) {
  if (e.type == EVT_MAG_BAD) { setState(FAULT); return; }
  if (e.type == EVT_BTN_BOTH_DOWN) { setState(WAIT_START); return; }

  if (e.type == EVT_WAYPOINT_REACHED) {
    // waypoint things
    return;
  }
}

void update_FAULT(Event e) {
  if (e.type == EVT_BTN_BOTH_DOWN) setState(WAIT_START);
}

// this function will push events to the queue if needed
void updateScheduler() {

}

void dispatchEvents() {
  Event e;
  while (popEvent(e)) {
    switch (g_state) {
      case BOOT:        update_BOOT(e); break;
      case CALIB_SERVO: break;  // implementing servo calibration
      case CALIB_MAG: break;    // implementing mag calibration
      case WAIT_START: update_WAIT_START(e); break;
      case RUN_NAV:   update_RUN_NAV(e);  break;
      case FAULT:   update_FAULT(e); break;
    }
  }
}

/// main setup/loop
void loop() {
  readInputs();
  updateScheduler();
  dispatchEvents();
  // applyOutputs(); optional loop, can handle within each event if needed
}