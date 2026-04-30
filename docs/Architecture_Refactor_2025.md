# Architecture Refactor: Veto/MotorPolicy → Safety Module Split

## What Changed

### DELETED (Old Architecture)
- `include/control/Veto.h` / `src/control/Veto.cpp`
- `include/control/MotorPolicy.h` / `src/control/MotorPolicy.cpp`
- `include/control/AutonomousOA.h` / `src/control/AutonomousOA.cpp`

### CREATED (New Architecture)
- `include/safety/ObstacleDetection.h` / `src/safety/ObstacleDetection.cpp`
- `include/safety/SafetyManager.h` / `src/safety/SafetyManager.cpp`
- `include/safety/MotionPolicy.h` / `src/safety/MotionPolicy.cpp`
- `include/control/AutonomousController.h` / `src/control/AutonomousController.cpp`

### MODIFIED (Integration Points)
- `src/main.cpp` — init/update new modules
- `src/control/InputWatchdog.cpp` — removed Veto coupling
- `src/control/MotorControl.cpp` — simplified to use MotionPolicy
- `src/control/MotorFault.cpp` — uses SafetyManager
- `src/control/ModeManager.cpp` — uses AutonomousController
- `src/input/BluetoothCommandParser.cpp` — includes updated

---

## The Bug We Fixed

**Problem:** Car wouldn't move when obstacle was at 45cm (soft block range)

**Root Cause:** Double-veto in `MotorPolicy::apply_directional_veto()`
```cpp
// OLD CODE (BROKEN)
if (Veto::has_reason(VetoReason::OA_FRONT)) {
    cmd.forward = 0.0f;  // ❌ Kills motion at 40cm
}
// Then authority scaling applies:
cmd.forward *= 0.5f;  // 0.0 * 0.5 = still 0.0
```

**Solution:** Separate stop zones from slow zones
- **Slow zone (40-50cm):** Only scale authority by 0.5x
- **Stop zone (15-25cm):** Force backoff or block motion

---

## New Architecture Benefits

### 1. Clear Separation of Concerns
- **ObstacleDetection** — sensor thresholds + hysteresis (one job)
- **SafetyManager** — system faults only (one job)
- **MotionPolicy** — decision logic (one job)
- **AutonomousController** — autonomous state machine (one job)

### 2. Single Truth Source
- No more checking: state + reasons + flags in 3 different modules
- Policy owns "can we move" decision
- Everyone else just reports their state

### 3. Better Hysteresis
- Old: One global 200ms timer, resets on ANY assertion
- New: Per-sensor timers, independent front/rear

### 4. Smarter Autonomous
- Old states: MOVING, SCANNING, ACTING, WAIT_RETRY
- New states: + CORNERED (try escape), + STUCK (give up, wait)
- Tries reversing, rotating in place when boxed in
- Won't blindly return to MOVING forward if path isn't clear

### 5. Simpler Integration
- InputWatchdog just reports expired state (no coupling)
- MotorControl has 1 call: `MotionPolicy::apply_safety(cmd)`
- SafetyManager polls InputWatchdog (not the other way around)

---

## How It Works Now

### Manual Mode
1. Bluetooth command arrives → `MotorControl::apply_command(cmd)`
2. `MotionPolicy::apply_safety(cmd)` enforces:
   - Emergency stop → zero everything
   - Input loss → zero everything
   - Front stop zone → backoff reverse (or block forward)
   - Rear stop zone → backoff forward (or block reverse)
   - Slow zones → scale by 0.5x
3. Mecanum mix → ramp → motors

### Autonomous Mode
1. `AutonomousController::update()` makes decisions
2. Sends commands through same `MotorControl::apply_command()` path
3. Policy enforces safety identically to manual mode
4. State machine handles: moving, scanning, executing, cornered, stuck

### Safety Faults
1. Emergency stop (`!` command) → `MotorFault::trigger()` → `SafetyManager::set_emergency_stop()`
2. Input loss → `InputWatchdog::is_expired()` → `SafetyManager::update()` detects it
3. Both cause `MotionPolicy` to return zero command

---

## Config Impact

No config changes needed. All existing thresholds still work:
- `FRONT_SLOW_ENTER_CM / EXIT_CM` — soft block range
- `FRONT_STOP_ENTER_CM / EXIT_CM` — hard block range
- `REAR_*` thresholds same
- `OA_SOFT_AUTHORITY` — still 0.5x scaling
- `OA_BACKOFF_SPEED` — still 0.3 for nudging away
- `VETO_CLEAR_HOLD_MS` — still 200ms hold before clearing

---

## Testing Checklist

### Basic Motion
- [ ] Forward/backward/strafe in open space
- [ ] Rotation in place
- [ ] Speed scaling (increase/decrease)

### Obstacle Avoidance (Manual)
- [ ] Approach obstacle slowly — car should scale down at 40cm
- [ ] Continue to 15cm — car should backoff reverse
- [ ] Try driving INTO obstacle at 10cm — backoff should engage
- [ ] Strafe near obstacle — should still work (not blocked)

### Autonomous Mode
- [ ] Forward driving until obstacle → scans → picks new direction
- [ ] Boxed into corner → tries reversing → tries rotating → gives up (STUCK)
- [ ] STUCK state retries every 5 seconds

### Safety Systems
- [ ] Emergency stop (`!`) → hard stop, latches
- [ ] Reset (`?`) → clears fault
- [ ] Bluetooth disconnect → input loss after 150ms → hard stop
- [ ] Bluetooth reconnect → resumes normal operation

---

## Migration Notes

If you have other modules that accessed Veto directly, they need updates:

**Old pattern:**
```cpp
#include "control/Veto.h"
if (Veto::get_state() == VetoState::CLEAR) { ... }
```

**New pattern:**
```cpp
#include "safety/SafetyManager.h"
#include "safety/ObstacleDetection.h"

SafetyState safety = SafetyManager::get_state();
Proximity front = ObstacleDetection::get_front();
if (safety == SafetyState::CLEAR && !front.in_stop_zone) { ... }
```
