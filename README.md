# 🤖 MicroMouse — ESP32 Competition Robot

A full PlatformIO project for a 16×16 micromouse competition robot using an ESP32.
Features: flood-fill maze solving · PID speed control · 3× VL53L0X ToF wall sensing · live web dashboard.

---

## 📁 Project Structure

```
MicroMoze/
├── platformio.ini          ← Board, libraries, LittleFS config
├── include/
│   ├── config.h            ← ⚙️  ALL pins & tunable constants  (edit me!)
│   ├── state.h             ← Shared g_state struct
│   ├── pid.h               ← PID controller interface
│   ├── encoder.h           ← Encoder interface
│   ├── motors.h            ← Motor driver interface
│   ├── tof.h               ← ToF sensor interface
│   ├── maze.h              ← Maze grid + flood-fill interface
│   ├── navigator.h         ← Cell-by-cell movement interface
│   └── webserver.h         ← Web server interface
├── src/
│   ├── main.cpp            ← setup() / loop() + state machine
│   ├── pid.cpp
│   ├── encoder.cpp
│   ├── motors.cpp
│   ├── tof.cpp
│   ├── maze.cpp
│   ├── navigator.cpp
│   └── webserver.cpp
└── data/
    └── index.html          ← Web dashboard (uploaded to flash)
```

---

## 🔌 Wiring Diagram

### Encoders  (⚠ need external 10 kΩ pull-ups — INPUT ONLY pins)

| Signal        | ESP32 GPIO |
|---------------|-----------|
| Right Enc A   | 34        |
| Right Enc B   | 35        |
| Left Enc A    | 36        |
| Left Enc B    | 39        |

### Motor Driver (e.g. DRV8833 or L298N)

| Signal       | ESP32 GPIO |
|--------------|-----------|
| Left PWM     | 13        |
| Left DIR1    | 14        |
| Left DIR2    | 27        |
| Right PWM    | 25        |
| Right DIR1   | 26        |
| Right DIR2   | 33        |

### VL53L0X ToF Sensors — I²C + XSHUT

| Signal       | ESP32 GPIO |
|--------------|-----------|
| I²C SDA      | 21        |
| I²C SCL      | 22        |
| Front XSHUT  | 4         |
| Left XSHUT   | 5         |
| Right XSHUT  | 18        |

> All three VL53L0X sensors share the same SDA/SCL lines.  
> VCC = 3.3 V.  Connect XSHUT of each sensor to its respective GPIO.

---

## ⚙️ Calibration (important!)

Open `include/config.h` and update these values for your robot:

```c
#define WHEEL_DIAMETER_MM  34.0f  // Measure your actual wheel
#define ENCODER_CPR        40     // Counts per revolution
#define TICKS_PER_90DEG    150L   // Run and measure empirically
```

**How to measure TICKS_PER_90DEG:**
1. Put the robot on the floor
2. Use web dashboard manual control → Turn Right once
3. Read encoder ticks from serial monitor (`pio device monitor`)
4. Update the constant and re-flash

---

## 🚀 Quick Start

### 1. Install PlatformIO
- Install [VS Code](https://code.visualstudio.com/)
- Install the PlatformIO extension

### 2. Build & Flash
```bash
# Compile and flash firmware
pio run --target upload

# Upload the web dashboard to flash (run once, or when HTML changes)
pio run --target uploadfs

# Open serial monitor
pio device monitor
```

### 3. Connect & Control
1. Connect your phone or laptop to WiFi: **`MicroMouse_Control`** / **`12345678`**
2. Open browser: **`http://192.168.1.1`**
3. The dashboard loads with live maze map and controls

---

## 🧠 How the Maze Solver Works

### Flood Fill (BFS)
1. Start with all 256 cells set to "distance = 255"
2. Run BFS from the goal cell outward — each cell gets its shortest-path distance
3. The robot always moves to the neighbour with the **lowest flood value**

### Exploration Loop
```
At each cell:
  1. Read ToF sensors → record walls in maze map
  2. Re-run flood fill with updated walls
  3. Choose direction with lowest flood value (open passages only)
  4. Move one cell in that direction
  5. Repeat until goal reached
```

### Fast Run
After exploration, the robot knows all walls → runs the same algorithm at **3× speed**.

---

## 📡 Web API Reference

| Endpoint | Description |
|----------|-------------|
| `GET /status` | Live JSON: speeds, ToF, position, mode, PID gains |
| `GET /maze_data` | Full maze: walls (16×16), flood values, robot position |
| `GET /command?cmd=explore` | Start maze exploration |
| `GET /command?cmd=return_home` | Return to start (0,0) |
| `GET /command?cmd=fast_run` | Speed run (requires prior exploration) |
| `GET /command?cmd=stop` | Emergency stop |
| `GET /command?cmd=reset` | Reset map and position |
| `GET /command?cmd=forward` | Manual: move one cell |
| `GET /command?cmd=turn_left` | Manual: turn 90° left |
| `GET /command?cmd=turn_right` | Manual: turn 90° right |
| `GET /set_speed?speed=300` | Set cruise speed (ticks/sec) |
| `GET /update_pid?kp=0.5&ki=0.05&kd=0.01` | Update PID gains |

---

## ⌨️ Dashboard Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` / `↑` | Move forward |
| `S` / `↓` | Move backward |
| `A` / `←` | Turn left |
| `D` / `→` | Turn right |
| `Space` | Stop |
| Click maze | Toggle flood-fill number overlay |

---

## 🔧 PID Tuning Guide

| Gain | Effect | Start value |
|------|--------|-------------|
| **Kp** | Responsiveness — too high → oscillation | 0.5 |
| **Ki** | Eliminates steady-state error — too high → windup | 0.05 |
| **Kd** | Dampens oscillation | 0.01 |

**Tuning procedure:**
1. Set Ki=0, Kd=0
2. Increase Kp until robot goes straight (slight oscillation OK)
3. Add Ki (small) to eliminate offset
4. Add Kd if oscillation remains
