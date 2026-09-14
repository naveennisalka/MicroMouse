#pragma once

// ═══════════════════════════════════════════════════════════════
//  PID Controller
//
//  One instance is created per wheel so there is no shared state.
//  Usage:
//    PID myPid;
//    pid_init(&myPid, 0.5f, 0.05f, 0.01f, 2000.0f);
//
//    // In a fixed-rate loop:
//    float output = pid_compute(&myPid, setpoint, measured, dt);
//    motor_set(constrain((int)output, -255, 255));
// ═══════════════════════════════════════════════════════════════

struct PID {
    float kp, ki, kd;
    float integral;       // Accumulated integral term
    float prevError;      // Error from last call (for derivative)
    float integralMax;    // Anti-windup clamp (±)
};

// Initialise a PID instance with given gains and anti-windup limit
void pid_init(PID* pid, float kp, float ki, float kd, float integralMax);

// Compute one PID step.
//   setpoint — desired value
//   measured — current value from sensor
//   dt       — time since last call in SECONDS
// Returns: raw output (caller should constrain before passing to motor)
float pid_compute(PID* pid, float setpoint, float measured, float dt);

// Zero the integral and prevError.
// Call whenever you change the setpoint abruptly to avoid integral kick.
void pid_reset(PID* pid);
