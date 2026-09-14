#pragma once

// ═══════════════════════════════════════════════════════════════
//  Motor Driver Module
//
//  Controls two DC motors via H-bridge (DIR1/DIR2 + PWM).
//    speed > 0  →  forward
//    speed < 0  →  backward
//    speed = 0  →  coast (both DIR pins LOW, PWM = 0)
//
//  Speed range: -255 … +255 (matches 8-bit PWM resolution)
// ═══════════════════════════════════════════════════════════════

// Configure LEDC PWM channels and direction GPIO pins
void motors_init();

// Set individual motor speeds (-255 … +255)
void motor_set_left(int speed);
void motor_set_right(int speed);

// Convenience: immediately stop both motors (PWM = 0)
void motors_stop();
