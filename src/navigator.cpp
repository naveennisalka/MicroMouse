/*
  navigator.cpp — Cell-by-Cell Movement Controller

  Moves the robot one maze cell at a time using encoder feedback.
  Works as a non-blocking state machine:

    ┌──────────────────────────────────────────────────────────────┐
    │  Caller                  Navigator                           │
    │  ──────                  ─────────                           │
    │  navigator_move_forward()  ← starts the move                │
    │                                                              │
    │  loop: navigator_update() → false (still moving)            │
    │  ...                                                         │
    │  loop: navigator_update() → false  ← NAV_SETTLING           │
    │        (motors already stopped; waiting for spin-down)       │
    │  loop: navigator_update() → TRUE  (done!)                   │
    │        maze_explore_step() ← picks next move                │
    └──────────────────────────────────────────────────────────────┘

  ### Bug fixes in this version ###

  1. DOUBLE-TURN BUG (turns 2× forward then 2× back):
     Previously navigator_update() returned true on the same cycle
     it stopped motors. Wheels were still spinning. The state machine
     fired the next command while the robot was still decelerating,
     encoder ticks kept growing, causing a cascade of spurious moves.
     Fix: NAV_SETTLING state holds motors off for NAV_SETTLE_CYCLES
     PID cycles before returning true.

  2. ROBOT NOT MOVING DURING TURNS:
     TURN_SPEED_TPS=150 with ff_gain=0.4 produced only 60 PWM —
     below the stiction threshold for in-place turns.
     Fix: MOTOR_MIN_TURN_PWM floor (see config.h) raises the tps
     target so the resulting PWM always exceeds the stall threshold.
     Also TURN_SPEED_TPS raised to 250 in config.h.

  3. STUCK TURN WATCHDOG:
     NAV_TURN_TIMEOUT_CYCLES (default 200 = 2 s) forces a turn to
     finish if encoders never confirm progress, preventing hangs.

  Position and heading live in g_state (state.h).
  Motor targets (g_state.targetLeftTps / targetRightTps) are
  set here; the PID in main.cpp converts them to PWM.
*/
#include "navigator.h"
#include "config.h"
#include "encoder.h"
#include "tof.h"
#include "state.h"
#include "motors.h"
#include <Arduino.h>

// ── Internal movement states ─────────────────────────────────────
enum NavCmd : uint8_t {
    NAV_IDLE,
    NAV_FORWARD,
    NAV_TURN_LEFT,
    NAV_TURN_RIGHT,
    NAV_TURN_AROUND,
    NAV_SETTLING       // Motors stopped; counting down before signalling done
};

static NavCmd currentCmd    = NAV_IDLE;
static NavCmd completedCmd  = NAV_IDLE; // Saved to apply heading update after settling
static long   tickTarget    = 0;
static long   startLeft     = 0;
static long   startRight    = 0;
static int    settleCycles  = 0;        // Countdown for NAV_SETTLING
static int    diagCycles    = 0;        // Encoder diagnostic print counter
static unsigned long moveStartMs = 0;  // Timestamp when current move began (millis)

// ── Direction offsets (indexed by heading: N=0 E=1 S=2 W=3) ────
static const int kDX[4] = {  0,  1,  0, -1 };
static const int kDY[4] = {  1,  0, -1,  0 };

// ── Helpers ───────────────────────────────────────────────────────
static long ticksTravelled() {
    long l = encoder_get_left();
    long r = encoder_get_right();
    return (abs(l - startLeft) + abs(r - startRight)) / 2L;
}

// Helper removed (min turn PWM is now handled cleanly in main.cpp's PID output)

// Hard-stop motors and enter the settling phase.
// The heading/position update happens AFTER settling (robot is still).
static void beginSettling(NavCmd justFinished) {
    g_state.motorsActive   = false;
    g_state.targetLeftTps  = 0;
    g_state.targetRightTps = 0;
    motors_stop();
    completedCmd = justFinished;
    settleCycles = NAV_SETTLE_CYCLES;
    currentCmd   = NAV_SETTLING;
}

// ── navigator_init ───────────────────────────────────────────────
void navigator_init() {
    currentCmd   = NAV_IDLE;
    completedCmd = NAV_IDLE;
    settleCycles = 0;
    diagCycles   = 0;
    moveStartMs  = 0;
}

// ── Movement starters ─────────────────────────────────────────────
void navigator_move_forward() {
    startLeft    = encoder_get_left();
    startRight   = encoder_get_right();
    tickTarget   = TICKS_PER_CELL;
    moveStartMs  = millis();
    diagCycles   = 0;
    currentCmd   = NAV_FORWARD;
    g_state.motorsActive = true;
}

void navigator_turn_left() {
    startLeft    = encoder_get_left();
    startRight   = encoder_get_right();
    tickTarget   = TICKS_PER_90DEG;
    moveStartMs  = millis();
    diagCycles   = 0;
    currentCmd   = NAV_TURN_LEFT;
    g_state.motorsActive = true;
    Serial.printf("[Nav] Turn-Left  start  L=%ld R=%ld  tickTarget=%ld  timeLimit=%dms\n",
                  startLeft, startRight, tickTarget, TURN_TIME_90_MS);
}

void navigator_turn_right() {
    startLeft    = encoder_get_left();
    startRight   = encoder_get_right();
    tickTarget   = TICKS_PER_90DEG;
    moveStartMs  = millis();
    diagCycles   = 0;
    currentCmd   = NAV_TURN_RIGHT;
    g_state.motorsActive = true;
    Serial.printf("[Nav] Turn-Right start  L=%ld R=%ld  tickTarget=%ld  timeLimit=%dms\n",
                  startLeft, startRight, tickTarget, TURN_TIME_90_MS);
}

void navigator_turn_around() {
    startLeft    = encoder_get_left();
    startRight   = encoder_get_right();
    tickTarget   = TICKS_PER_90DEG * 2L;
    moveStartMs  = millis();
    diagCycles   = 0;
    currentCmd   = NAV_TURN_AROUND;
    g_state.motorsActive = true;
    Serial.printf("[Nav] Turn-Around start L=%ld R=%ld  tickTarget=%ld  timeLimit=%dms\n",
                  startLeft, startRight, tickTarget, TURN_TIME_90_MS * 2);
}

// ── navigator_update — called every PID cycle ─────────────────────
bool navigator_update() {

    // ── IDLE ────────────────────────────────────────────────────────
    if (currentCmd == NAV_IDLE) return true;

    // ── SETTLING: motors off, wait for physical spin-down ──────────
    //  Heading/position update is applied here, after wheels are still.
    //  This is the fix for the double-turn bug: "done" is only returned
    //  once NAV_SETTLE_CYCLES PID cycles have elapsed after stopping.
    if (currentCmd == NAV_SETTLING) {
        g_state.motorsActive   = false;
        g_state.targetLeftTps  = 0;
        g_state.targetRightTps = 0;

        if (--settleCycles <= 0) {
            switch (completedCmd) {
                case NAV_FORWARD:
                    g_state.posX += kDX[g_state.heading];
                    g_state.posY += kDY[g_state.heading];
                    break;
                case NAV_TURN_LEFT:
                    g_state.heading = (g_state.heading + 3) % 4; // N→W→S→E→N
                    break;
                case NAV_TURN_RIGHT:
                    g_state.heading = (g_state.heading + 1) % 4; // N→E→S→W→N
                    break;
                case NAV_TURN_AROUND:
                    g_state.heading = (g_state.heading + 2) % 4;
                    break;
                default: break;
            }
            completedCmd = NAV_IDLE;
            currentCmd   = NAV_IDLE;
            return true;   // ← Only signal done AFTER robot is still
        }
        return false;
    }

    long progress = ticksTravelled();

    // ── Move forward (one cell) ──────────────────────────────────
    if (currentCmd == NAV_FORWARD) {

        // ── Wall centering using INNER diagonal sensors ───────────
        //
        // correction = (front_left - front_right) * gain
        //   positive → front_left > front_right → closer to right → steer left
        //   negative → front_left < front_right → closer to left  → steer right
        //
        float correction = 0.0f;
        bool hasLeft  = tof_wall_front_left();
        bool hasRight = tof_wall_front_right();

        if (hasLeft && hasRight) {
            correction = ((float)tof_front_left_mm - (float)tof_front_right_mm)
                         * WALL_ALIGN_GAIN;
        } else if (hasLeft) {
            correction = ((float)tof_front_left_mm - WALL_ALIGN_TARGET_MM)
                         * WALL_ALIGN_GAIN;
        } else if (hasRight) {
            correction = (WALL_ALIGN_TARGET_MM - (float)tof_front_right_mm)
                         * WALL_ALIGN_GAIN;
        }

        // ── Front-wall deceleration using OUTER sensors ───────────
        bool frontClose = tof_wall_front();
        float speedScale = frontClose ? 0.5f : 1.0f;

        g_state.targetLeftTps  = g_state.baseSpeed * speedScale - correction;
        g_state.targetRightTps = g_state.baseSpeed * speedScale + correction;
        g_state.motorsActive   = true;

        if (progress >= tickTarget) {
            // Defer position update to settling phase (robot still moving)
            beginSettling(NAV_FORWARD);
        }
        return false;
    }

    // ── Turn left (CCW 90°): right fwd, left back ─────────────────
    if (currentCmd == NAV_TURN_LEFT) {
        g_state.targetLeftTps  = -TURN_SPEED_TPS;
        g_state.targetRightTps =  TURN_SPEED_TPS;
        g_state.motorsActive   = true;


        // ── Encoder diagnostic ────────────────────────────────────
        if (ENCODER_DIAG_CYCLES > 0 && ++diagCycles >= ENCODER_DIAG_CYCLES) {
            diagCycles = 0;
            long elapsed = (long)(millis() - moveStartMs);
            Serial.printf("[Nav] Turn-Left  progress ticks=%ld  elapsed=%ldms  target_ticks=%ld  target_ms=%d\n",
                          ticksTravelled(), elapsed, tickTarget, TURN_TIME_90_MS);
        }

        // End condition: encoder ticks reached OR time elapsed (whichever first)
        bool ticksDone = (ticksTravelled() >= tickTarget && tickTarget > 0);
        bool timeDone  = (millis() - moveStartMs >= (unsigned long)TURN_TIME_90_MS);

        if (ticksDone || timeDone) {
            Serial.printf("[Nav] Turn-Left  DONE  ticks=%ld  elapsed=%ldms  by=%s\n",
                          ticksTravelled(), (long)(millis() - moveStartMs),
                          ticksDone ? "ENCODER" : "TIME");
            beginSettling(NAV_TURN_LEFT);
        }
        return false;
    }

    // ── Turn right (CW 90°): left fwd, right back ─────────────────
    if (currentCmd == NAV_TURN_RIGHT) {
        g_state.targetLeftTps  =  TURN_SPEED_TPS;
        g_state.targetRightTps = -TURN_SPEED_TPS;
        g_state.motorsActive   = true;


        if (ENCODER_DIAG_CYCLES > 0 && ++diagCycles >= ENCODER_DIAG_CYCLES) {
            diagCycles = 0;
            long elapsed = (long)(millis() - moveStartMs);
            Serial.printf("[Nav] Turn-Right progress ticks=%ld  elapsed=%ldms  target_ticks=%ld  target_ms=%d\n",
                          ticksTravelled(), elapsed, tickTarget, TURN_TIME_90_MS);
        }

        bool ticksDone = (ticksTravelled() >= tickTarget && tickTarget > 0);
        bool timeDone  = (millis() - moveStartMs >= (unsigned long)TURN_TIME_90_MS);

        if (ticksDone || timeDone) {
            Serial.printf("[Nav] Turn-Right DONE  ticks=%ld  elapsed=%ldms  by=%s\n",
                          ticksTravelled(), (long)(millis() - moveStartMs),
                          ticksDone ? "ENCODER" : "TIME");
            beginSettling(NAV_TURN_RIGHT);
        }
        return false;
    }

    // ── Turn around (180°) ────────────────────────────────────────
    if (currentCmd == NAV_TURN_AROUND) {
        g_state.targetLeftTps  =  TURN_SPEED_TPS;
        g_state.targetRightTps = -TURN_SPEED_TPS;
        g_state.motorsActive   = true;

        if (ENCODER_DIAG_CYCLES > 0 && ++diagCycles >= ENCODER_DIAG_CYCLES) {
            diagCycles = 0;
            long elapsed = (long)(millis() - moveStartMs);
            Serial.printf("[Nav] Turn-Around progress ticks=%ld  elapsed=%ldms  target_ticks=%ld  target_ms=%d\n",
                          ticksTravelled(), elapsed, tickTarget, TURN_TIME_90_MS * 2);
        }

        bool ticksDone = (ticksTravelled() >= tickTarget && tickTarget > 0);
        bool timeDone  = (millis() - moveStartMs >= (unsigned long)(TURN_TIME_90_MS * 2));

        if (ticksDone || timeDone) {
            Serial.printf("[Nav] Turn-Around DONE ticks=%ld  elapsed=%ldms  by=%s\n",
                          ticksTravelled(), (long)(millis() - moveStartMs),
                          ticksDone ? "ENCODER" : "TIME");
            beginSettling(NAV_TURN_AROUND);
        }
        return false;
    }

    return false;
}

bool navigator_is_done() {
    return currentCmd == NAV_IDLE;
}

void navigator_stop() {
    g_state.motorsActive   = false;
    g_state.targetLeftTps  = 0;
    g_state.targetRightTps = 0;
    motors_stop();
    currentCmd   = NAV_IDLE;
    completedCmd = NAV_IDLE;
    settleCycles = 0;
    diagCycles   = 0;
}
