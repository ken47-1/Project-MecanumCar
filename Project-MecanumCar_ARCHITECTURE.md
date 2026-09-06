# Project MecanumCar Architecture

## Overview

Bluetooth-controlled Mecanum wheel robot with autonomous obstacle avoidance. Arduino Uno + Adafruit Motor Shield V2 (TB6612FNG motor driver + PCA9685 PWM controller), modular C++ firmware on the Arduino Framework, dual HC-SR04 ultrasonic sensors with servo turret, real-time safety subsystem with independent front/rear obstacle detection.

## Why This Project Exists

This project is a complete rewrite of the **ZYC0044 Mini Mecanum Wheel Car** kit firmware.

### The Original Kit

The kit (available from ZHIYI Technology) includes:
- A single Arduino sketch (`MINI_Mecanum_Wheel_Car.ino`) that controls the car with a L293D motor driver and a 74HC595 shift register.
- One HC‑SR04 ultrasonic sensor for obstacle detection.
- An HM‑10 BLE module for Bluetooth control.
- Basic direction commands: Forward, Back, Left, Right, Turn Left, Turn Right, and Stop.
- Speed adjustment via `+` and `-` characters (10‑step increments).
- A simple obstacle‑avoidance routine: if an object is closer than 20 cm, the car backs up, turns left, and continues forward.

### What Was Missing

The original code works, but it has severe limitations for real‑world use:
- **Blocking delays** – `delay()` stops all other tasks, making the car unresponsive during maneuvers.
- **Flat structure** – everything is in one file with global variables; no modularity.
- **No safety system** – no watchdog, no input loss detection, no emergency stop.
- **Minimal obstacle logic** – only one sensor, no hysteresis, and a fixed 20 cm threshold.
- **No autonomous navigation** – no pathfinding; the car only backs up and turns left.
- **No motor ramping** – motors start and stop instantly, causing jerky motion.
- **Single‑board only** – designed for Arduino Uno R3; no support for R4.
- **Hardcoded values** – speeds, distances, and pin assignments are scattered and not configurable.

### The Rewrite

This firmware rebuilds the kit from the ground up with:
- **Modular C++ architecture** – separate modules for communication, control, input, safety, and sensors.
- **Non‑blocking timing** – all delays use `millis()`, so the car stays responsive.
- **Real‑time safety** – a watchdog, input loss detection, and a latching emergency stop.
- **Autonomous navigation** – a state machine with pathfinding (MOVING → SCANNING → SPINNING → BACKING_UP → STUCK).
- **Dual ultrasonic sensors** – front (servo‑mounted, 5‑position sweep) and rear.
- **Sensor filtering** – exponential moving average (EMA) and hysteresis for stable readings.
- **Motor ramping** – smooth acceleration and deceleration (400 ms up, 200 ms down).
- **Dual‑board support** – works on Arduino Uno R3 and R4 Minima/WiFi.
- **Fully configurable** – all settings are in `Config.h`, `HardwareConfig.h`, and `DebugConfig.h`.
- **Real feedback** – speed gauge, step mode, and debug output.

The same hardware now delivers autonomous driving, reliable obstacle avoidance, and safe operation – all in a clean, maintainable codebase.

### The Result

A professional-grade firmware for a hobbyist robot kit. The same hardware, now capable of autonomous driving, obstacle avoidance with pathfinding, and safe operation. Compatible with both Arduino Uno R3 and the newer R4 Series boards.

## Firmware Architecture

### Module Organization

**Comms** (`src/comms/`)
- `Comms` — Serial output multiplexing (debug mirror + Bluetooth feedback) with support for HC-05 (STATE pin optional) and HC-06 modules. `Comms::is_connected()` uses STATE pin when enabled, otherwise assumes always connected.
- `MultiPrint` — Fans output to two channels simultaneously
- Handles message formatting and transmission

**Control** (`src/control/`)
- `MotorHardware` — Singleton hardware ownership of motors; raw PWM interface to AFMS V2
- `MotorControl` — Applies motion commands to motors via MotorHardware
- `MotorRamp` — Acceleration/deceleration curves (400ms accel, 200ms decel)
- `MotorFault` — Fault state tracking and latching
- `ModeManager` — Manual vs. Autonomous mode switching with state tracking
- `AutonomousController` — Autonomous state machine (MOVING -> SCANNING -> SPINNING -> BACKING_UP -> STUCK) with servo sweep and pathfinding
- `MotionCommand` — Data structure for motion intents (speed, direction)

**Input** (`src/input/`)
- `BluetoothCommandParser` — Parses ASCII commands from Bluetooth Serial module
- `BluetoothButtonInput` — Maps button commands (W/A/S/D/Q/E/Z/C/J/L/X) to motion intents
- `BluetoothSpeedAuthority` — Handles speed step control (%+, %-, %R/%N/%F)
- `BluetoothSystemCommands` — Handles system commands (!, ?, 0, 1)
- `InputWatchdog` — Bluetooth keepalive; auto-stop on signal loss (150ms timeout)

**Safety** (`src/safety/`)
- `ObstacleDetection` — Sensor thresholds with EMA filtering (alpha=0.35) and hysteresis
  - Front: SLOW (40-50cm), STOP (15-25cm)
  - Rear: SLOW (40-50cm), STOP (15-25cm)
  - Returns `Proximity` struct: `in_slow_zone`, `in_stop_zone`, `distance_cm`
- `SafetyManager` — System fault aggregation (emergency stop, input loss)
- `MotionPolicy` — Decision logic for motion vetoes
  - Applies emergency stop override
  - Applies input loss (watchdog timeout) override
  - Applies obstacle detection scaling (0.5x in slow zone, block in stop zone)
  - Independent front/rear logic — can reverse when front blocked

**Sensors** (`src/sensors/`)
- `Ultrasonic` — HC-SR04 wrapper with EMA-filtered readings
  - `get_front_distance_cm()` — EMA-filtered front sensor
  - `get_rear_distance_cm()` — EMA-filtered rear sensor
  - `get_front_distance_raw_cm()` — Raw single ping (used during servo sweeps)
  - `get_rear_distance_raw_cm()` — Raw single ping
  - Dropout handling: `dist==0` treated as 999cm (assume clear)
- `DirectionalScan` — Servo sweep logic and multi-angle distance measurement
  - Sweeps 5 positions: LEFT (0°), FRONT_LEFT (45°), CENTER (90°), FRONT_RIGHT (135°), RIGHT (180°) [mirrored mounting]
  - Returns `SweepResult` with distances and `clear_mask` (bitmask of clear directions)
  - Dynamic settle times based on angle change: 45°=200ms, 90°=300ms, 135°=500ms (configurable)
- `BatteryVoltage` — 0-25V voltage sensor reader (Rev 2)
  - `get_voltage()` — Returns filtered voltage (EMA alpha=0.1)
  - `is_low()` — Returns true if voltage below `BATTERY_WARNING_VOLTAGE`
  - `report()` — Sends `*V{voltage}V*` telemetry every `BATTERY_REPORT_INTERVAL_MS`
  - Tracks minimum voltage seen for critical E-stop at `BATTERY_CRITICAL_VOLTAGE` (6.0V)

**Main Loop** (`src/main.cpp`)
1. `BluetoothCommandParser::handle(input_watchdog)` — Receive and dispatch commands
2. `input_watchdog.update()` — Check for signal loss (only when moving)
3. `ObstacleDetection::update()` — Poll ultrasonic sensors, apply hysteresis
4. `SafetyManager::update()` — Aggregate faults (E-STOP, INPUT_LOSS, BATTERY_CRITICAL)
5. `BatteryVoltage::report()` — Send voltage telemetry (every 2s)
6. `AutonomousController::update(input_watchdog)` — Execute autonomous state machine
7. `MotorRamp::update()` — Apply ramping curves
8. `MotorControl::update()` — Write PWM to motors

### Flowchart

```mermaid
graph TB
    subgraph Input["Input Layer"]
        BT[BluetoothCommandParser]
        WD[InputWatchdog]
    end

    subgraph Sensors["Sensor Layer"]
        US[Ultrasonic Sensors<br>HC-SR04 front/rear]
        DS[DirectionalScan<br>Servo sweep]
        BV[BatteryVoltage]
    end

    subgraph Safety["Safety Layer"]
        OD[ObstacleDetection<br>EMA + hysteresis]
        SM[SafetyManager<br>E-STOP + INPUT_LOSS]
        MP[MotionPolicy<br>Veto logic]
    end

    subgraph Control["Control Layer"]
        MC[MotorControl<br>Motion commands]
        MR[MotorRamp<br>Accel/decel curves]
        MH[MotorHardware<br>AFMS V2]
        AM[AutonomousController<br>State machine]
        MF[MotorFault]
        MD[ModeManager]
    end

    BT --> MC
    WD --> SM
    US --> OD
    DS --> AM
    BV --> SM
    OD --> MP
    SM --> MP
    MP --> MC
    AM --> MC
    MC --> MR
    MR --> MH
    MF --> MC
    MD --> AM
```

## Autonomous State Machine

### States (AutonomousController)

**MOVING**
- Drive forward at `AUTO_SPEED` (600 per-mille = 60%)
- Continuously monitor front/rear obstacles via `ObstacleDetection::get_front()` / `get_rear()`
- Transition to SCANNING on obstacle detection

**SCANNING**
- Stop at obstacle (front distance < `FRONT_STOP_ENTER_CM`)
- Servo sweeps 5 positions via `DirectionalScan::start_sweep()`
- Measure distance at each position (raw readings, bypasses EMA)
- Evaluate `SweepResult::clear_mask` (bitmask of clear directions)
- Transition to SPINNING, rotating toward clearest direction

**SPINNING**
- Rotate toward clearest direction
- Duration: `AUTO_SPIN_DIAGONAL_MS` (500ms, 45° adjustments) or `AUTO_SPIN_SIDE_MS` (1000ms, 90° adjustments)
- Time-based rotation (not feedback-driven, due to sensor unreliability mid-rotation)
- Transition to MOVING after rotation complete

**BACKING_UP**
- Escape maneuver: reverse at reduced speed
- Used when all directions blocked (CORNERED logic)
- Reverse escape (max 1 second or until rear is blocked)
- Transition back to SCANNING after escape

**STUCK**
- All directions blocked (distance <threshold in all 5 positions)
- Wait `AUTO_RETRY_WAIT_MS` (2000ms)
- Retry by transitioning back to MOVING
- If retries exhausted, give up and wait for user intervention

### Navigation Logic Flow

1. MOVING: forward motion, continuous monitoring
2. Obstacle detected -> SCANNING: servo sweep
3. Evaluate clear_mask from sweep results
4. Pick direction with max distance -> SPINNING: rotate to that direction
5. Resume MOVING after rotation
6. If all zones blocked -> BACKING_UP: escape attempt
7. BACKING_UP -> SCANNING: retry
8. If too many retries -> STUCK: wait and retry

```mermaid
flowchart TD
    A[MOVING] -->|hit obstacle| B[SCANNING]
    B -->|path found| C[SPINNING]
    C -->|rotation complete| A
    B -->|all blocked| D[BACKING_UP]
    D -->|max attempts| E[STUCK]
    D -->|retry| B
    E -->|wait & retry| B
```

## Control Protocol

### Command Format (ASCII, no framing)

Single-character commands sent one at a time or batched.

**Movement** (no prefix)
- `W` — Forward
- `S` — Backward
- `A` — Strafe left
- `D` — Strafe right
- `Q` — Forward-left diagonal
- `E` — Forward-right diagonal
- `Z` — Backward-left diagonal
- `C` — Backward-right diagonal
- `J` — Spin left (CCW)
- `L` — Spin right (CW)
- `X` — Soft stop (non-latching, also used as idle heartbeat)

**Speed Control** (`%` prefix)
- `%+` — Increase speed (apply `SPEED_STEP_NORMAL`)
- `%-` — Decrease speed (apply `SPEED_STEP_NORMAL`)
- `%R` — Set rough step mode (10% increments)
- `%N` — Set normal step mode (5% increments)
- `%F` — Set fine step mode (1% increments)

**System** (single char)
- `!` — Emergency stop (latching fault)
- `?` — Reset / clear emergency stop
- `1` — Autonomous mode ON
- `0` — Autonomous mode OFF
- `T` — Toggle arc turn mode (Fixed <-> Speed-Dependent)
  - **Fixed:** Always 0.5x rotation when moving
  - **Speed-Dependent:** Tighter turns at low speed, wider arcs at high speed

### Feedback (Robot -> App)

- `*G[value]*` — Speed gauge; value is 0–1000 (PWM 0–4095 scaled)
- `*%[mode]*` — Step mode; `Fine`, `Normal`, or `Rough`

### Watchdog Behavior

- **Always active:** Watchdog runs continuously, reset by any valid command.
- **Timeout:** 150ms (`INPUT_WATCHDOG_TIMEOUT_MS`)
- **Trigger:** No valid command received within timeout
- **Action:** InputWatchdog asserts INPUT_LOSS -> SafetyManager blocks motion
- **Recovery:** Any valid command clears INPUT_LOSS and resumes operation
- **Idle command:** `X` is a soft stop that also resets the watchdog when the robot is idle.

## Safety Subsystem

**Notes**
- Obstacle avoidance is enabled by default (`ENABLE_OBSTACLE_AVOIDANCE = 1` in Config.h). Disable it manually if desired.
- CONNECTION_LOSS only applies when `ENABLE_HC05_STATE_PIN` is enabled in `HardwareConfig.h` (off by default). Enable it manually if using HC-05.

### Watchdog

- **Timeout:** 150ms (`INPUT_WATCHDOG_TIMEOUT_MS`)
- **Trigger:** No valid command received within timeout
- **Action:** InputWatchdog notifies SafetyManager, which asserts INPUT_LOSS flag
- **Result:** MotorPolicy blocks all motion until `?` (reset) command
- **Emergent design:** Idle `X` command resets timeout without requiring dedicated keepalive frame

### Safety States (Priority Order)

| Priority | State | Trigger | Recovery |
|---|---|---|---|
| 1 | EMERGENCY_STOP | `!` command, battery critical, or hardware fault | Manual reset (`?`) |
| 2 | CONNECTION_LOSS | HC-05 STATE pin LOW | Re-pair/reconnect |
| 3 | INPUT_LOSS | Watchdog timeout while moving | Auto-resume on command |
| 4 | CLEAR | Normal operation | — |

**Note:** CONNECTION_LOSS only applies when `ENABLE_HC05_STATE_PIN` is enabled in `HardwareConfig.h` (off by default). Enable it manually if using HC-05.

### Obstacle Detection

**Thresholds** (configurable in Config.h)

| Zone | Distance | Action |
|---|---|---|
| Clear | >35cm | Full speed, no veto |
| Slow | 30–35cm | Scale to 0.5x speed (`OA_SOFT_AUTHORITY`) |
| Stop | 15–20cm | Block forward motion, force backoff if in motion |
| Blocked | <15cm | Don't enter this state |

**Hysteresis** (prevents oscillation at thresholds)
- Slow zone: 30cm entry -> 35cm exit
- Stop zone: 15cm entry -> 20cm exit

**EMA Filtering**
- Alpha = 0.35 (both front and rear)
- Balances responsiveness vs. noise suppression
- Formula: `filtered = filtered + alpha * (raw - filtered)`

**Independent Front/Rear**
- Front blocked -> can't drive forward, but can reverse
- Rear blocked -> can't reverse, but can drive forward
- Diagonal movement: both sensors must clear respective zones

**Sensor Dropout**
- `dist == 0` (raw sensor output) treated as 999cm (assume clear)
- Absorbs occasional sensor glitches without compensation math

### Emergency Stop

- Triggered by:
  - `!` command (user)
  - `BATTERY_CRITICAL_VOLTAGE` reached (6.0V minimum tracked)
  - Hardware fault (motor shield missing, etc.)
- Latches until `?` (reset) command
- **Immediate stop:** `MotorControl::hard_stop()` — no ramping
- Overrides all motion including autonomous mode
- SafetyManager tracks state; MotorPolicy enforces block

## Refactor History

### What Changed (Architecture Refactor 2026)

**Deleted (Old Architecture)**
- `include/control/Veto.h` / `src/control/Veto.cpp`
- `include/control/MotorPolicy.h` / `src/control/MotorPolicy.cpp`
- `include/control/AutonomousOA.h` / `src/control/AutonomousOA.cpp`

**Created (New Architecture)**
- `include/safety/ObstacleDetection.h` / `src/safety/ObstacleDetection.cpp`
- `include/safety/SafetyManager.h` / `src/safety/SafetyManager.cpp`
- `include/safety/MotionPolicy.h` / `src/safety/MotionPolicy.cpp`
- `include/control/AutonomousController.h` / `src/control/AutonomousController.cpp`

### The Bug That Was Fixed

**Problem:** Car wouldn't move when obstacle was at 45cm (soft block range)

**Root Cause:** Double-veto in `MotorPolicy::apply_directional_veto()`

```cpp
// OLD CODE (BROKEN)
if (Veto::has_reason(VetoReason::OA_FRONT)) {
    cmd.forward = 0.0f;  // Kills motion at 40cm
}
// Then authority scaling applies:
cmd.forward *= 0.5f;  // 0.0 * 0.5 = still 0.0
```

**Solution:** Separate stop zones from slow zones

- **Slow zone (40-50cm):** Only scale authority by 0.5x
- **Stop zone (15-25cm):** Force backoff or block motion

### New Architecture Benefits

1. **Clear Separation of Concerns** – Each safety module has one job
2. **Single Truth Source** – Policy owns "can we move" decision
3. **Better Hysteresis** – Per-sensor timers, independent front/rear
4. **Smarter Autonomous** – Added CORNERED and STUCK states
5. **Simpler Integration** – InputWatchdog just reports state; MotorControl has one safety call

## Safety Quick Reference

### Module Purposes (One-Line Each)

- **ObstacleDetection** — Reads sensors, applies thresholds with hysteresis, returns Proximity structs
- **SafetyManager** — Tracks emergency stop and input loss faults
- **MotionPolicy** — Decides what motion is allowed based on obstacles + safety state
- **AutonomousController** — State machine for autonomous driving with escape logic

### Key API Usage

**Check Obstacles**

```cpp
#include "safety/ObstacleDetection.h"

Proximity front = ObstacleDetection::get_front();
if (front.in_stop_zone) {
    // Too close! (within 15-25cm)
}
if (front.in_slow_zone) {
    // Approaching (within 40-50cm)
}
uint16_t distance = front.distance_cm;  // raw reading
```

**Check Safety State**

```cpp
#include "safety/SafetyManager.h"

SafetyState state = SafetyManager::get_state();
// Returns: CLEAR, INPUT_LOSS, or EMERGENCY_STOP
```

**Apply Motion Safely**

```cpp
#include "safety/MotionPolicy.h"

MotionCommand cmd = { 1.0f, 0.0f, 0.0f };  // forward
MotionCommand safe = MotionPolicy::apply_safety(cmd);
// safe.forward might be:
// - 0.0 (emergency stop / input loss)
// - 0.5 (slow zone scaling)
// - -0.25 (backoff from stop zone)
// - 1.0 (clear path)
```

### Data Flow

```mermaid
flowchart LR
    US[Ultrasonic::get_front_distance_cm] --> OD[ObstacleDetection::update]
    OD --> OD2[ObstacleDetection::get_front → Proximity]
    OD2 --> MP[MotionPolicy::apply_safety]
    MP --> MC[MotorControl::apply_command]
    MC --> Motors
Motors
```

### State Transitions (Autonomous)

```mermaid
flowchart LR
    A[MOVING] -->|hit obstacle| B[SCANNING]
    B -->|path found| C[SPINNING]
    C -->|rotation complete| A
    B -->|all blocked| D[BACKING_UP]
    D -->|max attempts| E[STUCK]
    D -->|retry| B
    E -->|wait & retry| B
```

### Debug Output

Enable in `DebugConfig.h`:

```cpp
#define DEBUG_ENABLED  1   // Master toggle

#if DEBUG_ENABLED   // EDIT BELOW
    #define COMMS_DEBUG_MIRROR  1   // Echo commands to debug serial
    #define DEBUG_COMMS         0   // Log Bluetooth communication
    #define DEBUG_MOTOR_RAMP    0   // Print ramp calculations
    #define DEBUG_OA_REASON     1   // Print veto reasons
    #define DEBUG_OA_SCALE      1   // Print speed scaling
    #define DEBUG_SENSORS       1   // Print sensor readings
    #define DEBUG_WATCHDOG      1   // Print watchdog resets
#else   // DO NOT EDIT BELOW
    #define COMMS_DEBUG_MIRROR  0
    #define DEBUG_COMMS         0
    #define DEBUG_WATCHDOG      0
    #define DEBUG_SENSORS       0
    #define DEBUG_MOTOR_RAMP    0
    #define DEBUG_OA_REASON     0
    #define DEBUG_OA_SCALE      0
#endif
```

### Migration from Old Code

| Old (Veto) | New (Safety Modules) |
|---|---|
| `Veto::get_state()` | `SafetyManager::get_state()` + `ObstacleDetection::get_front()` |
| `Veto::is_faulted()` | `SafetyManager::get_state() == SAFETY_EMERGENCY_STOP` |
| `Veto::is_input_loss()` | `SafetyManager::get_state() == SAFETY_INPUT_LOSS` |
| `Veto::is_front_stop_active()` | `ObstacleDetection::get_front().in_stop_zone` |
| `MotorPolicy::intent_scale()` | Built into `MotionPolicy::apply_safety()` |
| `MotorPolicy::apply_directional_veto()` | Built into `MotionPolicy::apply_safety()` |

## Configuration

**Config files:** `include/config/` contains three files:

- `DebugConfig.h` – Debug output toggles
- `HardwareConfig.h` – Physical hardware presence (pins, sensors installed)
- `Config.h` – Software behavior (thresholds, timing, speed, features)

### Hardware Pins

- **Servo:** D10 (`SCAN_SERVO_PIN`)
- **Front Ultrasonic:** TRIG D11, ECHO D12
- **Rear Ultrasonic:** TRIG D8, ECHO D9
- **Bluetooth:** D0/D1 (Hardware Serial) — R3 uses Serial, R4 uses Serial1
- **Bluetooth STATE (HC-05 only):** D2 (enabled by default — disable in HardwareConfig.h for HC-06)
- **Battery Monitoring (Rev 2):** A0
- **Encoders (Rev 3):** D3 (FL), D4 (FR), D5 (RL), D6 (RR)

### Bluetooth Hardware

| Module | Connection | Pins | Notes |
|---|---|---|---|
| HC-05 | UART | D0 (RX), D1 (TX) | Supports STATE pin for connection detection (disabled by default) |
| HC-06 | UART | D0 (RX), D1 (TX) | No STATE pin — leave `ENABLE_HC05_STATE_PIN = 0` in HardwareConfig.h |

- **Default baud rate:** 9600
- **STATE pin (HC-05 only):** D2 (disabled by default, enable via `ENABLE_HC05_STATE_PIN` in HardwareConfig.h)
- **Pairing PIN:** HC-06: 1234 or 0000; HC-05: 1234

### Servo Angles (degrees)

- `SERVO_LEFT = 180`
- `SERVO_FRONT_LEFT = 135`
- `SERVO_CENTER = 90`
- `SERVO_FRONT_RIGHT = 45`
- `SERVO_RIGHT = 0`

### Obstacle Thresholds (cm)

- **Front Slow:** 40–50cm
- **Front Stop:** 15–25cm
- **Rear Slow:** 40–50cm
- **Rear Stop:** 15–25cm

### EMA Filter

- `ULTRASONIC_EMA_ALPHA_FRONT = 0.35`
- `ULTRASONIC_EMA_ALPHA_REAR = 0.35`

### Feature Flags

**Software Features** (Config.h)

- `ENABLE_INPUT_WATCHDOG = 1` (ON)
- `ENABLE_INPUT_BUTTONS = 1` (ON)
- `ENABLE_INPUT_JOYSTICK = 0` (OFF)
- `ENABLE_INPUT_SPEED_AUTHORITY = 1` (ON)
- `ENABLE_DIRECTIONAL_SCAN = 1` (ON)
- `ENABLE_OBSTACLE_AVOIDANCE = 1` (ON)
- `ENABLE_AUTONOMOUS_MODE = 0` (OFF)

**Hardware Presence** (HardwareConfig.h)

- `ENABLE_HC05_STATE_PIN = 0` (OFF — enable for HC-05)
- `ENABLE_ULTRASONIC_FRONT = 1` (ON)
- `ENABLE_ULTRASONIC_REAR = 1` (ON)
- `ENABLE_SERVO = 1` (ON)
- `ENABLE_BATTERY_MONITOR = 1` (ON)
- `ENABLE_ENCODERS = 0` (OFF — Rev 3 planned)

All modules are fully optional. Each can be enabled/disabled at compile time via flags in `Config.h` and `HardwareConfig.h`.

### Revision History

**Rev 2 (Current)**

- Double-deck chassis
- Battery monitoring (A0)
- New pin layout (ultrasonics moved to D8/D9 and D11/D12)

**Rev 3 (Planned)**

- 4x H206 optical encoders (D3, D4, D5, D6)
- Closed-loop PID speed control
- Encoder-based odometry

### Watchdog & Input

- `INPUT_WATCHDOG_TIMEOUT_MS = 150`

- **Motion-aware:** Watchdog active only when robot is moving
- `JOYSTICK_DEADZONE = 30.0`
- `JOYSTICK_INPUT_MAX = 127.0`

### Servo & Scan

- `SCAN_SERVO_SETTLE_MS = 500` (per position)

### Obstacle Avoidance

- `OA_CLEAR_HOLD_MS = 200` (hysteresis hold time)
- `OA_SOFT_AUTHORITY = 0.5` (speed scale in slow zone)
- `OA_BACKOFF_SPEED = 0.25` (nudge-away backoff speed)

### Autonomous Mode

- `AUTO_SPEED = 600` (per-mille, 60%)
- `AUTO_RETRY_WAIT_MS = 2000` (wait before retrying when cornered)
- `AUTO_SPIN_DIAGONAL_MS = 500` (45° rotation time)
- `AUTO_SPIN_SIDE_MS = 1000` (90° rotation time)

### Battery Monitoring

- `BATTERY_WARNING_VOLTAGE = 7.0V` — Warning threshold, prints alert
- `BATTERY_CRITICAL_VOLTAGE = 6.0V` — Triggers emergency stop
- `BATTERY_EMA_ALPHA = 0.1` — Smoothing filter for voltage readings
- `BATTERY_MIN_DECAY_RATE = 0.01V/s` — Min voltage recovery rate
- `BATTERY_REPORT_INTERVAL_MS = 2000` — Telemetry interval (2 seconds)
- Tracks minimum voltage seen to catch sag under load
- Telemetry format:

- `*V{voltage}V*` — Filtered voltage (e.g., `*V7.72V*`)
- `*M{voltage}V*` — Minimum voltage (e.g., `*M7.70V*`)
- Warning cooldowns:

- `BATTERY_WARNING_COOLDOWN_MS = 10000` (10s between warnings)
- `BATTERY_CRITICAL_COOLDOWN_MS = 5000` (5s between critical alerts)

### Drive Behavior

- **Speed:** MIN (200), MAX (1000), DEFAULT (1000) per-mille
- **Speed steps:** ROUGH (100 = 10%), NORMAL (50 = 5%), FINE (10 = 1%)
- **Motor ramp:** UP (400ms), DOWN (200ms)
- **Turn ratio:** 1/2 (half speed on one side for turns) — constant defined but not currently used in motion mix

### Motor Output

- `PWM_MAX = 4095` (AFMS V2, 12-bit)
- (Commented: PWM_MAX = 255 for AFMS V1, 8-bit)

## Design Patterns

### Non-Blocking Architecture

- No `delay()` anywhere in codebase
- All timing based on `millis()`
- Sensor reads polled on demand
- Commands processed asynchronously

### Hardware Ownership

- `MotorHardware` owns Motor Shield (singleton)
- `DirectionalScan` owns servo
- `Sensors` owns HC-SR04 instances
- Other modules get refs/pointers only, never own hardware

### Threshold Tuning Over Compensation

- Sensor inaccuracy (±2–4cm) handled by hysteresis zones + EMA filtering, not math
- Simpler, more robust across environments
- Scales without calibration

### Emergent Watchdog

- No dedicated keepalive frame needed
- Idle `X` command naturally resets timeout
- Reduces protocol overhead

### Code Layout Standard

Enforced via `docs/Code_Layout_Standard.md`:

- `.h` files: `#pragma once`, INCLUDES, TYPES, API namespace (public only)
- `.cpp` files: header, INCLUDES, INTERNAL STATE, INTERNAL HELPERS, PUBLIC API
- One module per file
- Namespaces always
- No `using namespace` in headers
- Full qualification for cross-module calls

## Performance

| Metric | Value |
|---|---|
| Command latency | <10ms |
| Motor response | <50ms |
| Ultrasonic sampling | ~50ms per sensor |
| Servo sweep | ~2.5 seconds (5 positions x 500ms settle) |
| Speed ramp | 400ms accel, 200ms decel |
| Watchdog timeout | 150ms |
| Max speed | ~1.5 m/s (depends on gearing and PWM_MAX scaling) |
| PWM resolution | 12-bit (0–4095, AFMS V2) |

## Current Status

- Core firmware complete and tested
- Autonomous state machine validated on test runs
- Bluetooth control stable with HC-06
- Obstacle avoidance tuned for typical indoor environments (currently disabled by default)
- Ready for deployment or further customization

## Known Limitations

- Arduino Uno has limited RAM (2KB) — keep code modular
- HC-SR04 is slow (~50ms per reading) — not real-time capable
- Servo sweep blocks briefly (~2.5s) during autonomous scan
- 150ms watchdog timeout is hardcoded; may need adjustment for other Bluetooth modules
- Time-based spin (not feedback-driven) due to sensor unreliability during rotation
- Turn ratio constants are defined but not active in the motion mix