/*
  motors.cpp — DC Motor Driver

  Drives two motors through an H-bridge (e.g. DRV8833 / L298N).
  Each motor has two direction pins (DIR1, DIR2) and a PWM pin.

  DIR1  DIR2  Effect
  ────  ────  ──────────
  HIGH  LOW   Forward
  LOW   HIGH  Backward
  LOW   LOW   Coast (free spin)
  HIGH  HIGH  Brake (short across motor) — NOT used here
*/
#include "motors.h"
#include "config.h"
#include <Arduino.h>

void motors_init() {
    // Direction control pins
    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);

    // Set up LEDC PWM channels
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_L_PWM, PWM_CH_L);
    ledcAttachPin(MOTOR_R_PWM, PWM_CH_R);

    motors_stop();
}

void motor_set_left(int speed) {
    if (speed >= 0) {
        digitalWrite(MOTOR_L_DIR1, HIGH);
        digitalWrite(MOTOR_L_DIR2, LOW);
        ledcWrite(PWM_CH_L, constrain(speed, 0, 255));
    } else {
        digitalWrite(MOTOR_L_DIR1, LOW);
        digitalWrite(MOTOR_L_DIR2, HIGH);
        ledcWrite(PWM_CH_L, constrain(-speed, 0, 255));
    }
}

void motor_set_right(int speed) {
    if (speed >= 0) {
        digitalWrite(MOTOR_R_DIR1, HIGH);
        digitalWrite(MOTOR_R_DIR2, LOW);
        ledcWrite(PWM_CH_R, constrain(speed, 0, 255));
    } else {
        digitalWrite(MOTOR_R_DIR1, LOW);
        digitalWrite(MOTOR_R_DIR2, HIGH);
        ledcWrite(PWM_CH_R, constrain(-speed, 0, 255));
    }
}

void motors_stop() {
    // Coast: all direction pins LOW, PWM = 0
    digitalWrite(MOTOR_L_DIR1, LOW);
    digitalWrite(MOTOR_L_DIR2, LOW);
    digitalWrite(MOTOR_R_DIR1, LOW);
    digitalWrite(MOTOR_R_DIR2, LOW);
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, 0);
}
