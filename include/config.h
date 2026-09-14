#pragma once

// ═══════════════════════════════════════════════════════════════
//  MicroMouse — Central Configuration
//  ► Edit this file to tune pins, speeds, and PID gains.
//  ► All other source files pull their constants from here.
// ═══════════════════════════════════════════════════════════════

// ── Encoder pins ────────────────────────────────────────────────
// ⚠ GPIO 34/35/36/39 are INPUT ONLY — no internal pull-ups!
//   Your PCB must have external 10 kΩ pull-up resistors on these.
#define ENC_PAR  34   // Right encoder channel A (interrupt source)
#define ENC_PBR  35   // Right encoder channel B (direction sense)
#define ENC_PAL  36   // Left  encoder channel A (interrupt source)
#define ENC_PBL  39   // Left  encoder channel B (direction sense)

// ── Motor driver pins ────────────────────────────────────────────
#define MOTOR_L_PWM   13
#define MOTOR_L_DIR1  14
#define MOTOR_L_DIR2  27
#define MOTOR_R_PWM   25
#define MOTOR_R_DIR1  26
#define MOTOR_R_DIR2  33

// ── PWM settings ─────────────────────────────────────────────────
#define PWM_FREQ   20000  // 20 kHz (above human hearing)
#define PWM_RES    8      // 8-bit → duty cycle 0–255
#define PWM_CH_L   0      // LEDC channel for left  motor
#define PWM_CH_R   1      // LEDC channel for right motor

// ── I²C bus (shared by all VL53L0X sensors) ──────────────────────
#define I2C_SDA  21
#define I2C_SCL  22

// ── ToF XSHUT pins ───────────────────────────────────────────────
//
//  Top-down view  (green arrow = forward direction)
//
//        FRONT ◄────────────────────────────
//
//        ┌──────────────────────────────┐  ┌──┐ ← RIGHT wheel
//        │    ╱ tof_front_right ↗      │  │  │   (top in image)
//        │   ╱  (45° right of fwd)     │  └──┘
//        │──────────────────────── ─── │
//        │   ╲  tof_front_left ↘      │  ┌──┐ ← LEFT wheel
//        │    ╲ (45° left of fwd)      │  │  │   (bottom in image)
//        └──────────────────────────────┘  └──┘
//
//  tof_right_mm      = outer right sensor (points forward, near right edge)
//  tof_front_right_mm = inner right diagonal (~45° right)
//  tof_front_left_mm  = inner left diagonal  (~45° left)
//  tof_left_mm       = outer left sensor  (points forward, near left edge)
//
//  Wall detection roles:
//    FRONT wall  → outer sensors (tof_right + tof_left)  — face straight ahead
//    RIGHT wall  → inner right diagonal (tof_front_right) — angled to pick up wall
//    LEFT wall   → inner left diagonal  (tof_front_left)  — angled to pick up wall
//    Centering   → compare (tof_front_left - tof_front_right) for yaw correction
//
#define TOF_XSHUT_RIGHT       16   // Outer right sensor  (XSHUT GPIO)
#define TOF_XSHUT_FRONT_RIGHT 17   // Inner right diagonal
#define TOF_XSHUT_FRONT_LEFT  5  // Inner left diagonal
#define TOF_XSHUT_LEFT        4  // Outer left sensor

// ── ToF I2C addresses (assigned at boot) ─────────────────────────
#define TOF_ADDR_RIGHT        0x30
#define TOF_ADDR_FRONT_RIGHT  0x31
#define TOF_ADDR_FRONT_LEFT   0x32
#define TOF_ADDR_LEFT         0x33

// ── Wall detection thresholds ────────────────────────────────────
//
//  Calibrated geometry (robot centred in a 180 mm cell):
//
//    ┌──────────────────────────── 180 mm ───────────────────────────────┐
//    │ LEFT wall                                           RIGHT wall    │
//    │◄──── 130 mm ────►┌──────────────────────┐◄──── 130 mm ──────────►│
//    │  (diag sensors)  │   🤖  (centered)     │   (diag sensors)       │
//    │                  └──────────────────────┘                         │
//    │                        ▼  80 mm  ▼                                │
//    │                    FRONT wall                                      │
//    └───────────────────────────────────────────────────────────────────┘
//
//  • tof_right_mm / tof_left_mm (outer, straight-ahead):
//      ~80 mm when robot is one half-cell from the front wall.
//      Trigger threshold set comfortably above that.
//
//  • tof_front_right_mm / tof_front_left_mm (inner 45° diagonals):
//      ~130 mm when robot is centred between side walls.
//      Use this as the centering / alignment target.
//
#define WALL_PRESENT_MM       90    // Outer sensors: front wall detected below 90 mm
                                    //  (robot centred ≈ 80 mm → 90 mm gives margin)
#define WALL_DIAG_PRESENT_MM  150   // Inner diagonals: side wall detected below 150 mm
                                    //  (robot centred ≈ 130 mm → 150 mm gives margin)
#define WALL_ALIGN_TARGET_MM  130   // Centering target for diagonal sensors (mm)
                                    //  = measured reading when perfectly centred
#define WALL_ALIGN_GAIN       0.3f  // Centering correction gain (tune if oscillating)

// ── Per-sensor calibration offsets ───────────────────────────────
//  Set via /calibrate_tof in the web UI; update after measurement.
//  Formula:  offset = knownDistance_mm − rawReading_mm
//  Leave as 0 until you have measured the actual offset.
#define TOF_OFFSET_RIGHT_MM        0
#define TOF_OFFSET_FRONT_RIGHT_MM  0
#define TOF_OFFSET_FRONT_LEFT_MM   0
#define TOF_OFFSET_LEFT_MM         0

// ── Maze dimensions & goal ───────────────────────────────────────
#define MAZE_SIZE     16   // Standard 16×16 micromouse maze
#define MAZE_GOAL_X    7   // Goal zone: centre 2×2 (cells 7,7–8,8)
#define MAZE_GOAL_Y    7
#define START_X        0
#define START_Y        0
#define START_HEADING  0   // 0=North, 1=East, 2=South, 3=West

// ── Physical / encoder calibration ───────────────────────────────
// ⚠ Measure on your actual robot and update these!
#define CELL_SIZE_MM       180    // One maze cell width = 180 mm
#define WHEEL_DIAMETER_MM  32.0f  // Wheel outer diameter in mm
#define ENCODER_CPR        40     // Encoder counts per full wheel revolution
// Derived: ticks per millimetre of travel
#define TICKS_PER_MM       (ENCODER_CPR / (3.14159f * WHEEL_DIAMETER_MM))
// Ticks for one full cell (straight)
#define TICKS_PER_CELL     ((long)(CELL_SIZE_MM * TICKS_PER_MM))
// Ticks for 90° spot turn — CALIBRATE this on the actual floor!
#define TICKS_PER_90DEG    150L

// ── Speed presets (ticks per second) ────────────────────────────
#define EXPLORE_SPEED_TPS  300.0f  // Slow — sensing walls safely
#define TURN_SPEED_TPS     250.0f  // In-place turn speed (raised so PWM > stiction floor)
#define FAST_SPEED_TPS     800.0f  // Speed run — known maze

// ── Motor PWM floor (stiction guard) ─────────────────────────────
// During in-place turns the PID-computed PWM can fall below the
// motor's stiction threshold and the robot won't move at all.
// Any |PWM| below this value is raised to this floor.
#define MOTOR_MIN_TURN_PWM  60    // Minimum |PWM| during turns (0-255)

// ── Motor trim (straight-line drift correction) ───────────────────
//
//  WHY ROBOTS DRIFT SIDEWAYS:
//    Every motor has slightly different friction, winding resistance,
//    and gear efficiency. At the same PWM value, one motor spins faster
//    than the other → robot curves toward the slower side.
//    When encoders work, the PID loop corrects this automatically.
//    Until encoders are fixed, use this static trim as compensation.
//
//  HOW TO USE:
//    1. Flash firmware and drive the robot forward on a straight line.
//    2. Watch which way it curves:
//         Curves LEFT  → left motor is SLOWER  → increase MOTOR_LEFT_TRIM
//         Curves RIGHT → right motor is SLOWER → increase MOTOR_RIGHT_TRIM
//    3. Change the appropriate value in steps of 5 and reflash.
//       Typical range needed: 5–30 PWM units.
//    4. Only one of the two should be non-zero at a time
//       (boost the slow side, leave the fast side at 0).
//
//  ⚠ This trim applies to FORWARD movement only (not turns).
//    Once your encoders are wired with proper pull-up resistors,
//    the PID loop will self-correct and these can be set back to 0.
//
#define MOTOR_LEFT_TRIM   0    // Extra PWM added to left  motor (+boost / -reduce)
#define MOTOR_RIGHT_TRIM  0    // Extra PWM added to right motor (+boost / -reduce)

// ── Navigator timing ──────────────────────────────────────────────
// Cycles (× PID_INTERVAL_MS) motors are kept off after a move so
// wheels spin down before the next command starts.
#define NAV_SETTLE_CYCLES       5    // 5 × 10 ms = 50 ms spin-down pause

// ── Time-based turn fallback ──────────────────────────────────────
//
//  WHY THIS EXISTS:
//    Encoder pins on ESP32 (GPIO 34/35/36/39) have NO internal pull-ups.
//    If your PCB lacks external 10kΩ pull-up resistors, encoder ISRs
//    never fire and tick-count stays 0 forever. Changing TICKS_PER_90DEG
//    then has zero effect — the robot always runs the full timeout.
//
//  HOW IT WORKS:
//    Each turn records its start time. The turn ends when EITHER:
//      (a) encoder ticks reach TICKS_PER_90DEG  ← used when encoders work
//      (b) elapsed time reaches TURN_TIME_90_MS ← fallback when encoders broken
//    Whichever condition is true first wins.
//
//  HOW TO CALIBRATE TURN_TIME_90_MS:
//    1. Flash this firmware.
//    2. Open Serial Monitor (115200 baud).
//    3. Click "Turn Left" in the web UI.
//    4. Watch the Serial output — it prints elapsed time when turn ends.
//    5. Measure whether the robot turned exactly 90°.
//       - Turned TOO MUCH (>90°): decrease TURN_TIME_90_MS.
//       - Turned TOO LITTLE (<90°): increase TURN_TIME_90_MS.
//    6. Reflash and repeat until exactly 90°.
//
//  ⚠ Start at 500 ms. A typical small robot at TURN_SPEED_TPS=250
//    needs 400–700 ms for a 90° spot turn on a smooth surface.
//
#define TURN_TIME_90_MS   500   // ms for a 90° in-place turn — CALIBRATE THIS

// Encoder diagnostic: print tick count to Serial every N cycles during turns.
// Set to 0 to disable. Use 50 (= every 500ms) to check if encoders fire.
#define ENCODER_DIAG_CYCLES  50

// ── PID defaults ────────────────────────────────────────────────
#define DEFAULT_KP          0.5f
#define DEFAULT_KI          0.05f
#define DEFAULT_KD          0.01f
#define PID_INTEGRAL_MAX  5000.0f  // Anti-windup clamp

// ── Main loop timing ─────────────────────────────────────────────
#define PID_INTERVAL_MS  10   // PID + navigation update rate (ms)

// ── WiFi access point ─────────────────────────────────────────────
#define WIFI_SSID  "MicroMouse_Control"
#define WIFI_PASS  "12345678"
