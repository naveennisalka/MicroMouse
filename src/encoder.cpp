/*
  encoder.cpp — Quadrature Encoder Driver

  Uses rising-edge interrupts on channel A (ENC_PAL / ENC_PAR).
  Channel B is sampled inside the ISR to determine direction:
    Left  encoder: B == LOW  → moving forward → ticks++
    Right encoder: B == HIGH → moving forward → ticks++
  (Flip these if your robot goes backward when commanded forward.)

  ⚠ GPIO 34/35/36/39 are input-only.  External 10 kΩ pull-ups
    on ENC_PA* and ENC_PB* are REQUIRED on your hardware.
*/
#include "encoder.h"
#include "config.h"
#include <Arduino.h>

// Volatile: modified in ISR, read in main loop
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ── ISRs (must live in IRAM to avoid flash cache misses) ────────
void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(ENC_PBL) == LOW)
        leftTicks++;
    else
        leftTicks--;
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(ENC_PBR) == HIGH)
        rightTicks++;
    else
        rightTicks--;
}

// ── Public API ──────────────────────────────────────────────────
void encoder_init() {
    pinMode(ENC_PAR, INPUT);
    pinMode(ENC_PBR, INPUT);
    pinMode(ENC_PAL, INPUT);
    pinMode(ENC_PBL, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_PAR), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_PAL), leftEncoderISR,  RISING);
}

// Disable interrupts briefly to get a consistent snapshot
long encoder_get_left() {
    noInterrupts();
    long t = leftTicks;
    interrupts();
    return t;
}

long encoder_get_right() {
    noInterrupts();
    long t = rightTicks;
    interrupts();
    return t;
}

void encoder_reset() {
    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();
}
