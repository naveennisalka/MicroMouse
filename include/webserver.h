#pragma once

// ═══════════════════════════════════════════════════════════════
//  Web Server Module
//
//  Starts a WiFi access point and serves:
//    GET /           → index.html (from LittleFS)
//    GET /status     → JSON telemetry (speeds, ToF, position, mode)
//    GET /maze_data  → JSON maze walls + flood values (for canvas)
//    GET /command    → ?cmd=explore|return|fast_run|stop|reset
//                      ?cmd=forward|backward|turn_left|turn_right
//    GET /set_speed  → ?speed=<ticks_per_sec>
//    GET /update_pid → ?kp=<f>&ki=<f>&kd=<f>
// ═══════════════════════════════════════════════════════════════

// Start WiFi AP + register all HTTP routes + begin server.
// Call once in setup() after everything else is initialised.
void webserver_init();
