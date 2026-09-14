#pragma once

// ═══════════════════════════════════════════════════════════════
//  Navigator Module
//
//  Manages cell-by-cell movement of the robot using encoder
//  feedback. Uses a non-blocking state machine so the web
//  server and PID loop keep running during movement.
//
//  How it works:
//    1. Call navigator_move_forward() (or a turn variant).
//    2. On every PID loop cycle, call navigator_update().
//    3. navigator_update() updates the speed targets in g_state
//       and returns true when the movement is complete.
//    4. When done, the maze-solving code picks the next move.
//
//  Position and heading are stored in g_state (state.h).
// ═══════════════════════════════════════════════════════════════

// Call once after motors and encoders are initialised
void navigator_init();

// ── Start a movement ─────────────────────────────────────────
// These functions are non-blocking — they just set up the move.
// Progress happens via repeated calls to navigator_update().
void navigator_move_forward();   // Advance one cell straight ahead
void navigator_turn_left();      // Rotate 90° counter-clockwise (in place)
void navigator_turn_right();     // Rotate 90° clockwise (in place)
void navigator_turn_around();    // Rotate 180° (in place)

// ── Progress update ──────────────────────────────────────────
// Call every PID cycle (every PID_INTERVAL_MS).
// Sets g_state.targetLeftTps / targetRightTps / motorsActive.
// Returns true when the current movement finishes.
bool navigator_update();

// True when no movement is in progress
bool navigator_is_done();

// Emergency stop — cancels any movement immediately
void navigator_stop();
