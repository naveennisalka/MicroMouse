#pragma once

// ═══════════════════════════════════════════════════════════════
//  Quadrature Encoder Module
//
//  Each motor has a two-channel (A + B) quadrature encoder.
//  We attach a RISING-edge interrupt on channel A and read
//  channel B to determine spin direction.
//
//  ⚠ GPIO 34/35/36/39 are INPUT ONLY — your board must have
//    external 10 kΩ pull-up resistors on these pins.
// ═══════════════════════════════════════════════════════════════

// Set up GPIO pins and attach interrupt handlers
void encoder_init();

// Atomic (interrupt-safe) tick reads
// Positive = forward, negative = backward
long encoder_get_left();
long encoder_get_right();

// Reset both tick counters to zero.
// Call this at the start of each measured movement so progress
// is tracked from 0.
void encoder_reset();
