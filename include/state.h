#pragma once
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════
//  Shared Robot State
//
//  g_state is the single source of truth for all runtime data.
//  It is defined once in main.cpp and accessed as `extern` by
//  navigator.cpp and webserver.cpp.
//
//  Rule: only main.cpp (state machine) and navigator.cpp
//  (movement controller) WRITE to this struct.
//  Everyone else just READS from it.
// ═══════════════════════════════════════════════════════════════

// Robot operating modes
enum RobotMode : uint8_t {
    MODE_IDLE      = 0,  // Waiting for web command
    MODE_EXPLORE   = 1,  // Flood-fill exploration
    MODE_RETURN    = 2,  // Returning to start (0,0)
    MODE_FAST_RUN  = 3,  // Optimised speed run on known map
    MODE_MANUAL    = 4   // Remote control via web UI
};

struct RobotState {
    // ── Speed control ────────────────────────────────────────
    float leftSpeedTps;    // Measured left  wheel speed (ticks/s)
    float rightSpeedTps;   // Measured right wheel speed (ticks/s)
    float targetLeftTps;   // PID setpoint  for left  wheel
    float targetRightTps;  // PID setpoint  for right wheel
    float baseSpeed;       // User-configurable cruise speed (tps)
    bool  motorsActive;    // false → PID bypassed, motors held off

    // ── PID parameters (writable from web UI) ────────────────
    float kp, ki, kd;

    // ── Sensor readings ──────────────────────────────────────────
    uint16_t tofLeft;        // Left side sensor (mm)
    uint16_t tofFrontLeft;   // Front-Left diagonal sensor (mm)
    uint16_t tofFrontRight;  // Front-Right diagonal sensor (mm)
    uint16_t tofRight;       // Right side sensor (mm)

    // ── Position & heading ───────────────────────────────────
    int posX, posY;     // Current cell (0,0 = start)
    int heading;        // 0=North  1=East  2=South  3=West

    // ── Mode & flags ─────────────────────────────────────────
    RobotMode mode;
    bool goalReached;   // Set when robot first reaches goal zone
    bool mazeKnown;     // Set after a complete exploration
};

// Declared here, defined in main.cpp
extern RobotState g_state;
