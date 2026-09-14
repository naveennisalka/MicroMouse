/*
  main.cpp — MicroMouse Entry Point & Robot State Machine

  ┌────────────────────────────────────────────────────────────┐
  │  SYSTEM OVERVIEW                                           │
  │                                                            │
  │  setup()                                                   │
  │    └─ Init encoders, motors, ToF, maze, web server        │
  │                                                            │
  │  loop()  [runs as fast as possible]                        │
  │    └─ Every PID_INTERVAL_MS (10 ms):                      │
  │         ① tof_update()           read 3 distance sensors  │
  │         ② update_speed_control() PID → motor PWM          │
  │         ③ navigator_update()     encoder progress check   │
  │              └─ if done → maze step (explore/return/fast) │
  │                                                            │
  │  AsyncWebServer runs on the WiFi task (background)        │
  │  and never blocks the main loop.                          │
  └────────────────────────────────────────────────────────────┘

  State machine (g_state.mode):

    MODE_IDLE      ─┐
    ▲               │ web: /command?cmd=explore
    │ goal reached  ▼
    │           MODE_EXPLORE ──────────────────────┐
    │               │ goal reached                  │ web: stop
    │               ▼                              ▼
    │           MODE_RETURN              MODE_IDLE (stopped)
    │               │ back at start
    │               ▼
    │           MODE_IDLE ──── web: fast_run ───► MODE_FAST_RUN
    └───────────────────────────────────────────────────────────
*/
#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "encoder.h"
#include "motors.h"
#include "tof.h"
#include "maze.h"
#include "navigator.h"
#include "webserver.h"
#include "pid.h"

// ═══════════════════════════════════════════════════════════════
//  Global state — accessible by webserver.cpp and navigator.cpp
// ═══════════════════════════════════════════════════════════════
RobotState g_state;

// ═══════════════════════════════════════════════════════════════
//  PID controllers (one per wheel)
// ═══════════════════════════════════════════════════════════════
static PID pidLeft;
static PID pidRight;

// Called by webserver.cpp when PID gains change
void pid_reset_all() {
    pid_reset(&pidLeft);
    pid_reset(&pidRight);
}

// ═══════════════════════════════════════════════════════════════
//  Speed Control  (called every PID_INTERVAL_MS)
// ═══════════════════════════════════════════════════════════════
static long prevLeftTicks  = 0;
static long prevRightTicks = 0;
static unsigned long prevTimeUs = 0;

static void update_speed_control() {
    unsigned long now = micros();
    float dt = (now - prevTimeUs) / 1000000.0f;
    if (dt <= 0.0f) return;

    // Measure actual wheel speeds (ticks per second)
    long lt = encoder_get_left();
    long rt = encoder_get_right();
    g_state.leftSpeedTps  = (lt - prevLeftTicks)  / dt;
    g_state.rightSpeedTps = (rt - prevRightTicks) / dt;
    prevLeftTicks  = lt;
    prevRightTicks = rt;
    prevTimeUs     = now;

    if (!g_state.motorsActive) {
        // PID disabled — force motors off and clear integrals
        motors_stop();
        pid_reset(&pidLeft);
        pid_reset(&pidRight);
        return;
    }

    // Sync PID gains from g_state (user may have updated via web)
    pidLeft.kp  = pidRight.kp  = g_state.kp;
    pidLeft.ki  = pidRight.ki  = g_state.ki;
    pidLeft.kd  = pidRight.kd  = g_state.kd;

    // ── Pure PID speed control ────────────────────────────────────────
    //  Matches the working test code exactly. No feed-forward is used.
    //
    //  WHY NO FEED-FORWARD:
    //    FF assumes both motors produce the same speed at the same PWM.
    //    In reality, motor mismatch means one motor spins faster than the
    //    other at the same PWM — FF bakes in that imbalance permanently.
    //    Pure PID detects the actual encoder speed and corrects each wheel
    //    independently, so the robot goes straight automatically.
    float leftPID  = pid_compute(&pidLeft,  g_state.targetLeftTps,  g_state.leftSpeedTps,  dt);
    float rightPID = pid_compute(&pidRight, g_state.targetRightTps, g_state.rightSpeedTps, dt);

    int leftPWM  = (int)leftPID;
    int rightPWM = (int)rightPID;

    // ── Minimum PWM floor for turns ──────────────────────────────────
    //  Pure PID can take a few cycles to ramp up from 0 PWM.
    //  During spot turns, apply a floor so both wheels start moving
    //  immediately without waiting for PID to build up enough output.
    bool isTurning = (g_state.targetLeftTps * g_state.targetRightTps < 0.0f);
    if (isTurning) {
        if (leftPWM  > 0 && leftPWM  < MOTOR_MIN_TURN_PWM)  leftPWM  =  MOTOR_MIN_TURN_PWM;
        if (leftPWM  < 0 && leftPWM  > -MOTOR_MIN_TURN_PWM) leftPWM  = -MOTOR_MIN_TURN_PWM;
        if (rightPWM > 0 && rightPWM < MOTOR_MIN_TURN_PWM)  rightPWM =  MOTOR_MIN_TURN_PWM;
        if (rightPWM < 0 && rightPWM > -MOTOR_MIN_TURN_PWM) rightPWM = -MOTOR_MIN_TURN_PWM;
    }

    motor_set_left( constrain(leftPWM,  -255, 255));
    motor_set_right(constrain(rightPWM, -255, 255));
}

// ═══════════════════════════════════════════════════════════════
//  Wall Detection using all 4 ToF sensors
//
//  Sensor → Absolute wall direction mapping (by heading):
//
//  Sensor       | Heading N | Heading E | Heading S | Heading W
//  ─────────────┼───────────┼───────────┼───────────┼──────────
//  FRONT_LEFT   | NW corner | NE corner | SE corner | SW corner
//  FRONT_RIGHT  | NE corner | SE corner | SW corner | NW corner
//  LEFT side    | West wall | North wall| East wall | South wall
//  RIGHT side   | East wall | South wall| West wall | North wall
//
//  Front wall detection: if BOTH diagonal sensors trigger → wall ahead
//  Left/right wall: use respective side sensors
// ═══════════════════════════════════════════════════════════════
static void sense_and_record_walls(int x, int y, int heading) {
    static const uint8_t wallBits[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

    // Front wall: triggered when both diagonal sensors see a close object
    // (a wall straight ahead reflects both beams)
    if (tof_wall_front())
        maze_set_wall(x, y, wallBits[heading]);

    // Left side wall
    if (tof_wall_left())
        maze_set_wall(x, y, wallBits[(heading + 3) % 4]);

    // Right side wall
    if (tof_wall_right())
        maze_set_wall(x, y, wallBits[(heading + 1) % 4]);

    // Extra: if only one diagonal triggers, it likely means a diagonal corner
    // — we can infer a partial wall but we ignore it here for simplicity.

    maze_mark_visited(x, y);
}

// ═══════════════════════════════════════════════════════════════
//  Shared move-dispatch helper
//  relTurn = (bestDir - currentHeading + 4) % 4
//    0 → straight  1 → right  2 → U-turn  3 → left
// ═══════════════════════════════════════════════════════════════
static void dispatch_move(uint8_t bestDir) {
    int relTurn = ((int)bestDir - g_state.heading + 4) % 4;

    switch (relTurn) {
        case 0: navigator_move_forward(); break;
        case 1: navigator_turn_right();   break;
        case 2: navigator_turn_around();  break;
        case 3: navigator_turn_left();    break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  MODE_EXPLORE — one step of the maze exploration algorithm
// ═══════════════════════════════════════════════════════════════
static void maze_explore_step() {
    int x = g_state.posX;
    int y = g_state.posY;

    // 1. Record what we can sense from here
    sense_and_record_walls(x, y, g_state.heading);

    // 2. Have we reached the goal?
    if (x == MAZE_GOAL_X && y == MAZE_GOAL_Y) {
        g_state.goalReached = true;
        g_state.mazeKnown   = true;
        g_state.mode        = MODE_IDLE;
        g_state.motorsActive = false;
        motors_stop();
        Serial.printf("[Maze] ★ GOAL REACHED at (%d,%d)!\n", x, y);
        return;
    }

    // 3. Re-run flood fill with all known walls so far
    maze_flood_fill(MAZE_GOAL_X, MAZE_GOAL_Y);

    // 4. Choose the open passage leading to the lowest flood value
    uint8_t bestDir = maze_best_direction(x, y, true);

    // 5. Command navigator to move in that direction
    Serial.printf("[Explore] (%d,%d) h=%d → dir=%d  flood=%d\n",
                  x, y, g_state.heading, bestDir, g_maze[x][y].flood);
    dispatch_move(bestDir);
}

// ═══════════════════════════════════════════════════════════════
//  MODE_RETURN — navigate back to start (0,0)
// ═══════════════════════════════════════════════════════════════
static void maze_return_step() {
    int x = g_state.posX;
    int y = g_state.posY;

    if (x == START_X && y == START_Y) {
        g_state.mode         = MODE_IDLE;
        g_state.motorsActive = false;
        motors_stop();
        Serial.println("[Nav] ★ Back at start!");
        return;
    }

    uint8_t bestDir = maze_best_direction(x, y, true);
    dispatch_move(bestDir);
}

// ═══════════════════════════════════════════════════════════════
//  MODE_FAST_RUN — speed run on the known maze
// ═══════════════════════════════════════════════════════════════
static void maze_fast_step() {
    int x = g_state.posX;
    int y = g_state.posY;

    if (x == MAZE_GOAL_X && y == MAZE_GOAL_Y) {
        g_state.mode         = MODE_IDLE;
        g_state.motorsActive = false;
        motors_stop();
        Serial.println("[Fast] ★ GOAL reached (fast run)!");
        return;
    }

    uint8_t bestDir = maze_best_direction(x, y, true);
    dispatch_move(bestDir);
}

// ═══════════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n═══════════════════════════════");
    Serial.println("  MicroMouse Booting...");
    Serial.println("═══════════════════════════════");

    // ── Initialise shared state ──────────────────────────────────
    g_state = {};  // zero-initialise all fields
    g_state.baseSpeed = EXPLORE_SPEED_TPS;
    g_state.kp        = DEFAULT_KP;
    g_state.ki        = DEFAULT_KI;
    g_state.kd        = DEFAULT_KD;
    g_state.mode      = MODE_IDLE;
    g_state.posX      = START_X;
    g_state.posY      = START_Y;
    g_state.heading   = START_HEADING;

    // ── Hardware ─────────────────────────────────────────────────
    encoder_init();
    Serial.println("[OK] Encoders");

    motors_init();
    Serial.println("[OK] Motors");

    navigator_init();
    Serial.println("[OK] Navigator");

    // ── PID controllers ──────────────────────────────────────────
    pid_init(&pidLeft,  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, PID_INTEGRAL_MAX);
    pid_init(&pidRight, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, PID_INTEGRAL_MAX);
    Serial.println("[OK] PID controllers");

    // ── Maze ─────────────────────────────────────────────────────
    maze_init();
    maze_flood_fill(MAZE_GOAL_X, MAZE_GOAL_Y);
    Serial.printf("[OK] Maze (%dx%d), goal=(%d,%d)\n",
                  MAZE_SIZE, MAZE_SIZE, MAZE_GOAL_X, MAZE_GOAL_Y);

    // ── ToF sensors ──────────────────────────────────────────────
    if (!tof_init()) {
        Serial.println("[WARN] ToF init failed — running without wall sensing!");
    }

    // ── Web server (must be last) ─────────────────────────────────
    webserver_init();

    prevTimeUs = micros();

    Serial.println("═══════════════════════════════");
    Serial.printf("  Ready!  Connect to WiFi: %s\n", WIFI_SSID);
    Serial.printf("  Open browser: http://192.168.1.1\n");
    Serial.println("═══════════════════════════════\n");
}

// ═══════════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════════
void loop() {
    static unsigned long lastPID = 0;
    unsigned long now = millis();

    if (now - lastPID < PID_INTERVAL_MS) return;  // not yet time
    lastPID = now;

    // ── 1. Update sensor readings ────────────────────────────────
    tof_update();
    g_state.tofLeft       = tof_left_mm;
    g_state.tofFrontLeft  = tof_front_left_mm;
    g_state.tofFrontRight = tof_front_right_mm;
    g_state.tofRight      = tof_right_mm;

    // ── 2. PID speed control (writes to motors via motor_set_*) ──
    update_speed_control();

    // ── 3. Navigator: advance current movement ───────────────────
    bool moveDone = navigator_update();

    // ── 4. State machine: trigger next step when move completes ──
    if (moveDone && g_state.mode != MODE_IDLE && g_state.mode != MODE_MANUAL) {
        // navigator_update() already waited NAV_SETTLE_CYCLES before
        // returning true, so the robot is physically still here.
        switch (g_state.mode) {
            case MODE_EXPLORE:  maze_explore_step(); break;
            case MODE_RETURN:   maze_return_step();  break;
            case MODE_FAST_RUN: maze_fast_step();    break;
            default: break;
        }
    }
}
