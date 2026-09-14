/*
  tof.cpp — 4× VL53L0X Time-of-Flight Sensor Driver

  Actual sensor layout (top-down, robot moves toward green arrow = LEFT):

         FRONT ◄─────────────────────────
                                           ┌──┐ ← RIGHT wheel
      tof_right      ──────────────────►   │  │   (top of image)
      (outer right)                        └──┘
      tof_front_right  ╱─────────────►
      (inner 45° right)
                       ╲─────────────►
      tof_front_left   (inner 45° left)    ┌──┐ ← LEFT wheel
                                           │  │   (bottom of image)
      tof_left       ──────────────────►   └──┘
      (outer left)

  Wall detection logic:
    ┌────────────────────────────────────────────────────────┐
    │  FRONT wall  ← outer sensors (tof_right, tof_left)    │
    │               both pointing mostly straight forward    │
    │                                                        │
    │  RIGHT wall  ← inner right diagonal (tof_front_right) │
    │  LEFT  wall  ← inner left  diagonal (tof_front_left)  │
    │                                                        │
    │  Centering   ← (tof_front_left - tof_front_right)     │
    │               positive = closer to right → steer left │
    └────────────────────────────────────────────────────────┘

  Boot address-assignment (XSHUT trick):
    All XSHUT LOW → sensors silent
    1. Enable RIGHT        → assign 0x30
    2. Enable FRONT_RIGHT  → assign 0x31
    3. Enable FRONT_LEFT   → assign 0x32
    4. Enable LEFT         → assign 0x33
*/
#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ── Sensor instances ──────────────────────────────────────────────
static Adafruit_VL53L0X sRight;
static Adafruit_VL53L0X sFrontRight;
static Adafruit_VL53L0X sFrontLeft;
static Adafruit_VL53L0X sLeft;

// ── Public readings ───────────────────────────────────────────────
uint16_t tof_right_mm       = 0;
uint16_t tof_front_right_mm = 0;
uint16_t tof_front_left_mm  = 0;
uint16_t tof_left_mm        = 0;

// ── Private: read one sensor reliably ────────────────────────────
//
//  RangeStatus codes (VL53L0X):
//    0 = Valid measurement         ← only accept this
//    1 = Sigma limit fail          (measurement too noisy)
//    2 = Signal fail               (too little return signal)
//    3 = Min range early convergence
//    4 = Phase fail / out of range ← previously the only one filtered
//
//  We take the median of 3 consecutive samples to reduce noise spikes.
//
static uint16_t readOneSingle(Adafruit_VL53L0X& s) {
    VL53L0X_RangingMeasurementData_t m;
    s.rangingTest(&m, false);
    // Accept ONLY status 0 (valid); everything else is treated as "no object"
    return (m.RangeStatus == 0) ? m.RangeMilliMeter : 0;
}

static uint16_t readOne(Adafruit_VL53L0X& s) {
    uint16_t a = readOneSingle(s);
    uint16_t b = readOneSingle(s);
    uint16_t c = readOneSingle(s);
    // Sort three values and return the median
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b; // median
}

// ── Init — enable sensors one by one, assign unique addresses ─────
bool tof_init() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // Step 1: disable all sensors
    pinMode(TOF_XSHUT_RIGHT,       OUTPUT);
    pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT);
    pinMode(TOF_XSHUT_FRONT_LEFT,  OUTPUT);
    pinMode(TOF_XSHUT_LEFT,        OUTPUT);
    digitalWrite(TOF_XSHUT_RIGHT,       LOW);
    digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
    digitalWrite(TOF_XSHUT_FRONT_LEFT,  LOW);
    digitalWrite(TOF_XSHUT_LEFT,        LOW);
    delay(10);

    // Step 2: Outer RIGHT sensor → 0x30
    digitalWrite(TOF_XSHUT_RIGHT, HIGH);
    delay(10);
    if (!sRight.begin(TOF_ADDR_RIGHT)) {
        Serial.println("[ToF] ERROR: Right (outer) sensor not detected!");
        return false;
    }
    Serial.printf("[ToF] Right (outer)       OK  addr=0x%02X\n", TOF_ADDR_RIGHT);

    // Step 3: Inner FRONT-RIGHT diagonal → 0x31
    digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
    delay(10);
    if (!sFrontRight.begin(TOF_ADDR_FRONT_RIGHT)) {
        Serial.println("[ToF] ERROR: Front-Right (inner) sensor not detected!");
        return false;
    }
    Serial.printf("[ToF] Front-Right (inner) OK  addr=0x%02X\n", TOF_ADDR_FRONT_RIGHT);

    // Step 4: Inner FRONT-LEFT diagonal → 0x32
    digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
    delay(10);
    if (!sFrontLeft.begin(TOF_ADDR_FRONT_LEFT)) {
        Serial.println("[ToF] ERROR: Front-Left (inner) sensor not detected!");
        return false;
    }
    Serial.printf("[ToF] Front-Left (inner)  OK  addr=0x%02X\n", TOF_ADDR_FRONT_LEFT);

    // Step 5: Outer LEFT sensor → 0x33
    digitalWrite(TOF_XSHUT_LEFT, HIGH);
    delay(10);
    if (!sLeft.begin(TOF_ADDR_LEFT)) {
        Serial.println("[ToF] ERROR: Left (outer) sensor not detected!");
        return false;
    }
    Serial.printf("[ToF] Left (outer)        OK  addr=0x%02X\n", TOF_ADDR_LEFT);

    Serial.println("[ToF] All 4 sensors ready");
    return true;
}

// ── Update all readings ───────────────────────────────────────────
//  After reading, apply per-sensor calibration offsets from config.h.
//  Offset = KnownDistance - RawReading (measured during /calibrate_tof).
//  Values are clamped to 0 to prevent uint16 underflow wrap-around.
void tof_update() {
    auto applyOffset = [](uint16_t raw, int offset) -> uint16_t {
        int corrected = (int)raw + offset;
        return (corrected > 0) ? (uint16_t)corrected : 0;
    };

    tof_right_mm       = applyOffset(readOne(sRight),      TOF_OFFSET_RIGHT_MM);
    tof_front_right_mm = applyOffset(readOne(sFrontRight), TOF_OFFSET_FRONT_RIGHT_MM);
    tof_front_left_mm  = applyOffset(readOne(sFrontLeft),  TOF_OFFSET_FRONT_LEFT_MM);
    tof_left_mm        = applyOffset(readOne(sLeft),       TOF_OFFSET_LEFT_MM);
}

// ── Wall detection ────────────────────────────────────────────────

// FRONT wall: outer sensors (mostly straight-ahead) see it first
bool tof_wall_front() {
    bool r = tof_right_mm > 0 && tof_right_mm < WALL_PRESENT_MM;
    bool l = tof_left_mm  > 0 && tof_left_mm  < WALL_PRESENT_MM;
    return r || l;   // either outer sensor triggering = front wall
}

// RIGHT wall: inner right diagonal picks up the wall to the right
bool tof_wall_right() {
    return tof_front_right_mm > 0 && tof_front_right_mm < WALL_DIAG_PRESENT_MM;
}

// LEFT wall: inner left diagonal picks up the wall to the left
bool tof_wall_left() {
    return tof_front_left_mm > 0 && tof_front_left_mm < WALL_DIAG_PRESENT_MM;
}

// Aliases used in navigator.cpp for front deceleration
bool tof_wall_front_left()  { return tof_front_left_mm  > 0 && tof_front_left_mm  < WALL_DIAG_PRESENT_MM; }
bool tof_wall_front_right() { return tof_front_right_mm > 0 && tof_front_right_mm < WALL_DIAG_PRESENT_MM; }
