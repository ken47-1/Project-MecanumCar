# Quick Reference: Safety Architecture

## Module Purposes

**ObstacleDetection**
- Reads front and rear HC-SR04 ultrasonic sensors, applies configurable distance thresholds (slow zone 40-50cm, stop zone 15-25cm) with independent hysteresis timers per sensor to prevent oscillation. Uses EMA filtering (alpha = 0.35) to smooth raw sensor readings. Returns `Proximity` structs containing distance, slow zone flag, and stop zone flag.

**SafetyManager**
- Aggregates fault states from multiple sources: emergency stop latching (triggered by `!` command or `MotorFault`), input loss detection (from `InputWatchdog` timeout), and obstacle stop flags. Exports a single `SafetyState` enum: `CLEAR`, `INPUT_LOSS`, or `EMERGENCY_STOP`. Polled every loop to drive motion policy decisions.

**MotionPolicy**
- Consumes the current `MotionCommand` (forward, strafe, rotate) and applies all safety rules: zeroes motion on emergency stop or input loss, forces backoff (0.25× reverse speed) when front or rear stop zone is active, scales authority by 0.5× in slow zones, and preserves strafe and rotation even when forward/backward are blocked. Returns the final safe command for motor mixing.

**AutonomousController**
- State machine that drives the robot without user input. States: `MOVING` (forward at 60% speed), `SCANNING` (servo sweeps 5 positions to find clearest path), `SPINNING` (time‑based rotation toward chosen direction), `BACKING_UP` (short reverse escape when all paths are blocked), `STUCK` (retry after 2 seconds if cornered). Manual input instantly overrides and returns to `MANUAL` mode.
---

## Key API Usage

### Check Obstacles
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

### Check Safety State
```cpp
#include "safety/SafetyManager.h"

SafetyState state = SafetyManager::get_state();
// Returns: CLEAR, INPUT_LOSS, or EMERGENCY_STOP
```

### Apply Motion Safely
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

---

## Data Flow

```
Sensors::get_front_distance_cm()
    ↓
ObstacleDetection::update()  [applies hysteresis]
    ↓
ObstacleDetection::get_front()  → Proximity struct
    ↓
MotionPolicy::apply_safety(cmd)  [reads obstacles + safety]
    ↓
MotorControl::apply_command(safe_cmd)
    ↓
Motors
```

---

## State Transitions (Autonomous)

```
MOVING ──[hit obstacle]──> SCANNING
   ↑                           ↓
   │                    [path found]
   │                           ↓
   └────[clear ahead]──── SPINNING
                               ↓
                        [all blocked]
                               ↓
                          BACKING_UP ──[max attempts]──> STUCK
                               ↑                              ↓
                               └──────[retry]────────────────┘
```

---

## Debug Output

Enable in `Config.h`:
```cpp
#define DEBUG_SENSORS       1  // Prints "Front: 45 | Rear: 120"
#define DEBUG_OA_REASON     1  // Prints zone flags per sensor
#define DEBUG_OA_SCALE      1  // Prints policy decisions
#define DEBUG_WATCHDOG      1  // Prints watchdog resets
```

---

## Common Scenarios

### "Car won't move when obstacle is far away"
- Check if `DEBUG_OA_SCALE` shows authority = 0.0
- Verify thresholds: SLOW should be 40cm+, STOP should be 15cm+
- Old bug was directional veto firing in slow zone — NOW FIXED

### "Car crashes into obstacles"
- Check sensor readings with `DEBUG_SENSORS`
- Verify stop zones triggering: `DEBUG_OA_REASON`
- Check backoff engaging: `DEBUG_OA_SCALE` should show backoff

### "Autonomous gets stuck in corners"
- Normal! BACKING_UP state tries escape (reverse, rotate)
- After max attempts → STUCK (waits 2 seconds, retries)
- Manual input always overrides and exits autonomous

### "Input loss triggers randomly"
- App should send commands every 50ms (even 'X' idle heartbeat)
- Timeout is 150ms (3 missed packets)
- Check Bluetooth signal strength

---

## Migration from Old Code

| Old (Veto) | New (Safety Modules) |
|------------|----------------------|
| `Veto::get_state()` | `SafetyManager::get_state()` + `ObstacleDetection::get_front()` |
| `Veto::is_faulted()` | `MotorFault::active()` |
| `Veto::is_input_loss()` | `SafetyManager::get_state() == SAFETY_INPUT_LOSS` |
| `Veto::is_front_stop_active()` | `ObstacleDetection::get_front().in_stop_zone` |
| `MotorPolicy::intent_scale()` | Built into `MotionPolicy::apply_safety()` |
| `MotorPolicy::apply_directional_veto()` | Built into `MotionPolicy::apply_safety()` |

---

## Changes made:

| Original | Fixed |
|----------|-------|
| `OA_BACKOFF_SPEED = 0.3` (in description, not in code) | Removed the incorrect value; the code uses `0.25f` |
| `DEBUG_VETO_REASON` / `DEBUG_VETO_SCALE` | Changed to `DEBUG_OA_REASON` / `DEBUG_OA_SCALE` (actual flag names in `Config.h`) |
| `EXECUTING` state | Changed to `SPINNING` (actual state name in `AutonomousController.cpp`) |
| `CORNERED` state | Changed to `BACKING_UP` (actual state name, with retry logic handled there) |
| `retry 5s` | Changed to `2 seconds` (matches `AUTO_RETRY_WAIT_MS = 2000`) |
| `Veto::is_faulted()` → `SafetyManager::is_emergency_stop()` | Changed to `MotorFault::active()` (the actual fault source) |
| `SafetyManager::is_input_loss()` – no such method | Changed to direct state check `SafetyManager::get_state() == SAFETY_INPUT_LOSS` |