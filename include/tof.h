#pragma once
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════
//  Time-of-Flight Sensor Module  (4× VL53L0X)
//
//  Sensor layout (top-down view):
//
//         ┌─────────────────────────────┐
//         │  [FL 45°]      [FR 45°]    │
//         │                             │
//         │ [L]        🤖         [R]  │
//         └─────────────────────────────┘
//
//  LEFT        (GPIO XSHUT 4)  — side-facing, wall following
//  FRONT_LEFT  (GPIO XSHUT 5)  — 45° diagonal, front-left wall
//  FRONT_RIGHT (GPIO XSHUT 18) — 45° diagonal, front-right wall
//  RIGHT       (GPIO XSHUT 19) — side-facing, wall following
//
//  All 4 sensors share one I²C bus. Each is assigned a unique
//  address at boot using its XSHUT pin.
//
//  A reading of 0 means no object detected / sensor error.
// ═══════════════════════════════════════════════════════════════

// Initialise all four sensors (I²C + sequential address assignment).
// Returns false if any sensor is missing.
bool tof_init();

// Read all four sensors and update the variables below.
// Call once per PID cycle (every 10 ms).
void tof_update();

// ── Latest readings (mm) ─────────────────────────────────────────
extern uint16_t tof_left_mm;         // Left side sensor
extern uint16_t tof_front_left_mm;   // Front-Left diagonal sensor
extern uint16_t tof_front_right_mm;  // Front-Right diagonal sensor
extern uint16_t tof_right_mm;        // Right side sensor

// ── Wall detection helpers ────────────────────────────────────────
// Side sensors use WALL_PRESENT_MM threshold
bool tof_wall_left();
bool tof_wall_right();

// Diagonal sensors use WALL_DIAG_PRESENT_MM (shorter — angled mount)
bool tof_wall_front_left();
bool tof_wall_front_right();

// Convenience: wall directly ahead = either diagonal sensor triggered
bool tof_wall_front();
