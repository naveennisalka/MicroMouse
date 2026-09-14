/*
  pid.cpp — PID Controller Implementation

  Each wheel gets its own PID instance (created in main.cpp).
  The computation is a standard P + I + D with anti-windup clamping.
*/
#include "pid.h"
#include <Arduino.h>  // for constrain()

void pid_init(PID* pid, float kp, float ki, float kd, float integralMax) {
    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->integralMax = integralMax;
    pid_reset(pid);
}

float pid_compute(PID* pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    // ── Integral with anti-windup ──────────────────────────────
    pid->integral += error * dt;
    pid->integral  = constrain(pid->integral, -pid->integralMax, pid->integralMax);

    // ── Derivative ────────────────────────────────────────────
    float derivative = (error - pid->prevError) / dt;
    pid->prevError   = error;

    return (pid->kp * error)
         + (pid->ki * pid->integral)
         + (pid->kd * derivative);
}

void pid_reset(PID* pid) {
    pid->integral  = 0.0f;
    pid->prevError = 0.0f;
}
