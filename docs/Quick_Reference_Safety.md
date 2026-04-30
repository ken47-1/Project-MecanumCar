# Quick Reference: New Safety Architecture

## Module Purposes (One-Liner Each)

**ObstacleDetection** — Reads sensors, applies thresholds with hysteresis, returns Proximity structs
**SafetyManager** — Tracks emergency stop and input loss faults
**MotionPolicy** — Decides what motion is allowed based on obstacles + safety state
**AutonomousController** — State machine for autonomous driving with escape logic

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

if (SafetyManager::is_emergency_stop()) {
    // Latching fault active
}
```

### Apply Motion Safely
```cpp
#include "safety/MotionPolicy.h"

MotionCommand cmd = { 1.0f, 0.0f, 0.0f };  // forward
MotionCommand safe = MotionPolicy::apply_safety(cmd);
// safe.forward might be:
// - 0.0 (emergency stop / input loss)
// - 0.5 (slow zone scaling)
// - -0.3 (backoff from stop zone)
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
   └────[clear ahead]──── EXECUTING
                               ↓
                        [all blocked]
                               ↓
                           CORNERED ──[3 attempts]──> STUCK
                               ↑                         ↓
                               └──────[retry 5s]─────────┘
```

---

## Debug Output

Enable in `Config.h`:
```cpp
#define DEBUG_SENSORS       1  // Prints "Front: 45 | Rear: 120"
#define DEBUG_VETO_REASON   1  // Prints zone flags per sensor
#define DEBUG_VETO_SCALE    1  // Prints policy decisions
```

---

## Common Scenarios

### "Car won't move when obstacle is far away"
- Check if `DEBUG_VETO_SCALE` shows authority = 0.0
- Verify thresholds: SLOW should be 40cm+, STOP should be 15cm+
- Old bug was directional veto firing in slow zone — NOW FIXED

### "Car crashes into obstacles"
- Check sensor readings with `DEBUG_SENSORS`
- Verify stop zones triggering: `DEBUG_VETO_REASON`
- Check backoff engaging: `DEBUG_VETO_SCALE` should show "BACKOFF"

### "Autonomous gets stuck in corners"
- Normal! CORNERED state tries escape (reverse, rotate)
- After 3 attempts → STUCK (waits 5 seconds, retries)
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
| `Veto::is_faulted()` | `SafetyManager::is_emergency_stop()` |
| `Veto::is_input_loss()` | `SafetyManager::is_input_loss()` |
| `Veto::is_front_stop_active()` | `ObstacleDetection::get_front().in_stop_zone` |
| `MotorPolicy::intent_scale()` | Built into `MotionPolicy::apply_safety()` |
| `MotorPolicy::apply_directional_veto()` | Built into `MotionPolicy::apply_safety()` |
