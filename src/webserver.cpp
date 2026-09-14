/*
  webserver.cpp — WiFi Access Point + HTTP API

  Starts a WiFi AP so you can connect your phone/laptop directly
  to the robot (no router needed).

  Endpoints:
    GET /             → Serves data/index.html from LittleFS
    GET /status       → Live telemetry JSON (speeds, ToF, position, mode)
    GET /maze_data    → Full maze JSON (walls, flood values, robot position)
    GET /command      → ?cmd=<command>
    GET /set_speed    → ?speed=<ticks_per_sec>
    GET /update_pid   → ?kp=<f>&ki=<f>&kd=<f>

  Commands (?cmd=):
    explore           Start flood-fill maze exploration
    return_home       Navigate back to start (0,0)
    fast_run          Speed run on known maze
    stop              Stop all movement
    reset             Reset maze map + position to start
    forward           Manual: move forward one cell
    backward          Manual: move backward one cell
    turn_left         Manual: turn 90° left
    turn_right        Manual: turn 90° right
*/
#include "webserver.h"
#include "config.h"
#include "state.h"
#include "maze.h"
#include "navigator.h"
#include "pid.h"
#include "motors.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Arduino.h>

// ── Extern PID instances (defined in main.cpp) ───────────────────
extern void pid_reset_all();   // Resets both left & right PIDs

// ── Server ───────────────────────────────────────────────────────
static AsyncWebServer server(80);

// ── Mode names for Serial log ────────────────────────────────────
static const char* kModeNames[] = { "IDLE", "EXPLORE", "RETURN", "FAST_RUN", "MANUAL" };

// ── Helper: send plain "OK" 200 response ─────────────────────────
static void sendOK(AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "OK");
}

// ── /status — live telemetry ─────────────────────────────────────
static void handleStatus(AsyncWebServerRequest* req) {
    char buf[600];
    snprintf(buf, sizeof(buf),
        "{"
        "\"leftSpeed\":%.2f,"
        "\"rightSpeed\":%.2f,"
        "\"targetSpeed\":%.1f,"
        "\"tofLeft\":%u,"
        "\"tofFrontLeft\":%u,"
        "\"tofFrontRight\":%u,"
        "\"tofRight\":%u,"
        "\"posX\":%d,"
        "\"posY\":%d,"
        "\"heading\":%d,"
        "\"mode\":%d,"
        "\"goalReached\":%s,"
        "\"mazeKnown\":%s,"
        "\"kp\":%.4f,"
        "\"ki\":%.4f,"
        "\"kd\":%.4f"
        "}",
        g_state.leftSpeedTps,
        g_state.rightSpeedTps,
        g_state.baseSpeed,
        g_state.tofLeft,
        g_state.tofFrontLeft,
        g_state.tofFrontRight,
        g_state.tofRight,
        g_state.posX,
        g_state.posY,
        g_state.heading,
        (int)g_state.mode,
        g_state.goalReached ? "true" : "false",
        g_state.mazeKnown   ? "true" : "false",
        g_state.kp,
        g_state.ki,
        g_state.kd
    );
    req->send(200, "application/json", buf);
}

// ── /maze_data — full maze state for dashboard canvas ───────────
static void handleMazeData(AsyncWebServerRequest* req) {
    // Approximate size: 16*16*8 chars walls + 16*16*4 chars flood + overhead
    String json;
    json.reserve(MAZE_SIZE * MAZE_SIZE * 12 + 128);

    json = "{\"walls\":[";
    for (int y = 0; y < MAZE_SIZE; y++) {
        json += "[";
        for (int x = 0; x < MAZE_SIZE; x++) {
            json += String(g_maze[x][y].walls & 0x1F); // 5 bits: walls + visited
            if (x < MAZE_SIZE - 1) json += ",";
        }
        json += "]";
        if (y < MAZE_SIZE - 1) json += ",";
    }
    json += "],\"flood\":[";
    for (int y = 0; y < MAZE_SIZE; y++) {
        json += "[";
        for (int x = 0; x < MAZE_SIZE; x++) {
            json += String(g_maze[x][y].flood);
            if (x < MAZE_SIZE - 1) json += ",";
        }
        json += "]";
        if (y < MAZE_SIZE - 1) json += ",";
    }
    json += "],\"robotX\":";   json += g_state.posX;
    json += ",\"robotY\":";    json += g_state.posY;
    json += ",\"heading\":";   json += g_state.heading;
    json += ",\"goalX\":";     json += MAZE_GOAL_X;
    json += ",\"goalY\":";     json += MAZE_GOAL_Y;
    json += "}";

    req->send(200, "application/json", json);
}

// ── /command ─────────────────────────────────────────────────────
static void handleCommand(AsyncWebServerRequest* req) {
    if (!req->hasParam("cmd")) { sendOK(req); return; }
    String cmd = req->getParam("cmd")->value();

    Serial.printf("[Web] Command: %s\n", cmd.c_str());

    if (cmd == "explore") {
        // Reset maze map and start exploration
        maze_init();
        maze_flood_fill(MAZE_GOAL_X, MAZE_GOAL_Y);
        g_state.posX        = START_X;
        g_state.posY        = START_Y;
        g_state.heading     = START_HEADING;
        g_state.goalReached = false;
        g_state.baseSpeed   = EXPLORE_SPEED_TPS;
        g_state.mode        = MODE_EXPLORE;
        Serial.println("[Maze] Exploration started");

    } else if (cmd == "return_home") {
        // Flood fill toward start and navigate back
        maze_flood_fill(START_X, START_Y);
        g_state.baseSpeed = EXPLORE_SPEED_TPS;
        g_state.mode      = MODE_RETURN;
        Serial.println("[Maze] Returning to start");

    } else if (cmd == "fast_run") {
        if (!g_state.mazeKnown) {
            req->send(400, "text/plain", "Maze not known yet — explore first");
            return;
        }
        maze_flood_fill(MAZE_GOAL_X, MAZE_GOAL_Y);
        g_state.posX      = START_X;
        g_state.posY      = START_Y;
        g_state.heading   = START_HEADING;
        g_state.baseSpeed = FAST_SPEED_TPS;
        g_state.mode      = MODE_FAST_RUN;
        Serial.println("[Maze] Fast run started");

    } else if (cmd == "stop") {
        g_state.mode         = MODE_IDLE;
        g_state.motorsActive = false;
        g_state.targetLeftTps  = 0;
        g_state.targetRightTps = 0;
        navigator_stop();
        motors_stop();
        Serial.println("[Nav] STOPPED");

    } else if (cmd == "reset") {
        g_state.mode         = MODE_IDLE;
        g_state.motorsActive = false;
        g_state.posX         = START_X;
        g_state.posY         = START_Y;
        g_state.heading      = START_HEADING;
        g_state.goalReached  = false;
        g_state.mazeKnown    = false;
        navigator_stop();
        motors_stop();
        maze_init();
        maze_flood_fill(MAZE_GOAL_X, MAZE_GOAL_Y);
        Serial.println("[Maze] RESET");

    } else {
        // Manual movement commands
        g_state.mode = MODE_MANUAL;

        if (cmd == "forward") {
            navigator_move_forward();
        } else if (cmd == "backward") {
            navigator_turn_around();  // Turn then move
        } else if (cmd == "turn_left") {
            navigator_turn_left();
        } else if (cmd == "turn_right") {
            navigator_turn_right();
        }
    }

    sendOK(req);
}

// ── /set_speed ───────────────────────────────────────────────────
static void handleSetSpeed(AsyncWebServerRequest* req) {
    if (req->hasParam("speed")) {
        float spd = req->getParam("speed")->value().toFloat();
        spd = constrain(spd, 50.0f, 3000.0f);
        g_state.baseSpeed = spd;
        Serial.printf("[Web] Base speed → %.1f tps\n", spd);
    }
    sendOK(req);
}

// ── /update_pid ──────────────────────────────────────────────────
static void handleUpdatePID(AsyncWebServerRequest* req) {
    if (req->hasParam("kp")) g_state.kp = req->getParam("kp")->value().toFloat();
    if (req->hasParam("ki")) g_state.ki = req->getParam("ki")->value().toFloat();
    if (req->hasParam("kd")) g_state.kd = req->getParam("kd")->value().toFloat();

    // Reset integrals to prevent violent kick on new gains
    pid_reset_all();

    Serial.printf("[PID] kp=%.4f  ki=%.4f  kd=%.4f\n",
                  g_state.kp, g_state.ki, g_state.kd);
    sendOK(req);
}

// ── /calibrate_tof — raw sensor dump for calibration ────────────
//
//  Place the robot facing a wall at a KNOWN distance (e.g. 100 mm).
//  Open http://192.168.1.1/calibrate_tof in a browser.
//  The page shows both the raw readings AND the offset-corrected values.
//
//  To compute each sensor's offset:
//    offset = knownDistance - rawReading
//  Then update TOF_OFFSET_*_MM in config.h and reflash.
//
static void handleCalibrateTof(AsyncWebServerRequest* req) {
    // Re-read sensors freshly (tof_update() runs in the main loop but we
    // want values printed right now — use the most recent g_state copy).
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"note\":\"Place robot at a known distance from a wall, then compute: offset = knownDistance - raw\","
        "\"sensors\":{"
            "\"right\":{\"raw\":%u,\"corrected\":%u,\"offset_cfg\":%d},"
            "\"frontRight\":{\"raw\":%u,\"corrected\":%u,\"offset_cfg\":%d},"
            "\"frontLeft\":{\"raw\":%u,\"corrected\":%u,\"offset_cfg\":%d},"
            "\"left\":{\"raw\":%u,\"corrected\":%u,\"offset_cfg\":%d}"
        "}"
        "}",
        // raw = corrected - offset (reverse the offset to show raw)
        (uint16_t)(g_state.tofRight      > 0 ? (int)g_state.tofRight      - TOF_OFFSET_RIGHT_MM       : 0), g_state.tofRight,      TOF_OFFSET_RIGHT_MM,
        (uint16_t)(g_state.tofFrontRight > 0 ? (int)g_state.tofFrontRight - TOF_OFFSET_FRONT_RIGHT_MM : 0), g_state.tofFrontRight, TOF_OFFSET_FRONT_RIGHT_MM,
        (uint16_t)(g_state.tofFrontLeft  > 0 ? (int)g_state.tofFrontLeft  - TOF_OFFSET_FRONT_LEFT_MM  : 0), g_state.tofFrontLeft,  TOF_OFFSET_FRONT_LEFT_MM,
        (uint16_t)(g_state.tofLeft       > 0 ? (int)g_state.tofLeft       - TOF_OFFSET_LEFT_MM        : 0), g_state.tofLeft,       TOF_OFFSET_LEFT_MM
    );
    req->send(200, "application/json", buf);
}

// ── webserver_init ───────────────────────────────────────────────
void webserver_init() {
    // Mount LittleFS (holds data/index.html)
    if (!LittleFS.begin(true)) {
        Serial.println("[Web] LittleFS mount failed!");
    } else {
        Serial.println("[Web] LittleFS mounted OK");
    }

    // Start WiFi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(
        IPAddress(192, 168, 1, 1),
        IPAddress(192, 168, 1, 1),
        IPAddress(255, 255, 255, 0)
    );
    WiFi.softAP(WIFI_SSID, WIFI_PASS);

    Serial.printf("[WiFi] AP: %s  IP: %s\n", WIFI_SSID,
                  WiFi.softAPIP().toString().c_str());

    // ── Register routes ──────────────────────────────────────────
    // Static files (index.html) from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Dynamic API endpoints
    server.on("/status",        HTTP_GET, handleStatus);
    server.on("/maze_data",     HTTP_GET, handleMazeData);
    server.on("/command",       HTTP_GET, handleCommand);
    server.on("/set_speed",     HTTP_GET, handleSetSpeed);
    server.on("/update_pid",    HTTP_GET, handleUpdatePID);
    server.on("/calibrate_tof", HTTP_GET, handleCalibrateTof);

    // 404 fallback
    server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println("[Web] Server started on http://192.168.1.1");
}
