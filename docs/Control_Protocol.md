# Control Protocol Reference
App ⇄ Robot Communication Contract

> Documents ALL valid commands and feedback frames — no logic.
> Update this file when the app or firmware protocol changes.

---

## Command Format

```
[Command]           Discrete / system (no prefix)
[Prefix][Command]   Prefixed command
```

| Prefix | Domain |
|--------|--------|
| (none) | Movement / system |
| `%`    | Speed / step mode |

---

## Commands (App → Robot)

### Movement
Single-character, no prefix.

| Command | Action |
|---------|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Strafe left |
| `D` | Strafe right |
| `Q` | Forward-left |
| `E` | Forward-right |
| `Z` | Backward-left |
| `C` | Backward-right |
| `J` | Spin left (CCW) |
| `L` | Spin right (CW) |

---

### Speed Control (`%` prefix)

| Command | Action |
|---------|--------|
| `%+` | Increase speed |
| `%-` | Decrease speed |
| `%F` | Fine step mode |
| `%N` | Normal step mode |
| `%R` | Rough step mode |

---

### System

| Command | Action |
|---------|--------|
| `X` | Soft stop (non-latching) |
| `!` | Emergency stop (latching fault) |
| `?` | Reset fault |
| `1` | Autonomous mode ON |
| `0` | Autonomous mode OFF |
| `^` | Force watchdog feed (master keepalive) |
| `T` | Toggle arc turn mode (Fixed ↔ Speed-Dependent) |

**Notes:** HC-05 STATE pin (optional) detects physical disconnection. See `ENABLE_HC05_STATE_PIN` in Config.h. `T` toggles between Fixed (always 0.5× rotation) and Speed-Dependent (tighter at low speed, wider at high) arc turning. Prints current mode.

---

## Feedback (Robot → App)

| Frame | Description |
|-------|-------------|
| `*G[value]*` | Speed gauge — value is `0–1000` |
| `*%[mode]*` | Step mode — `Fine`, `Normal`, or `Rough` |
| `*V[value]V*` | Filtered battery voltage — e.g., `*V7.72V*` |
| `*M[value]V*` | Minimum battery voltage (with decay) — e.g., `*M7.70V*` |

### Battery Voltage

- **Filtered (`*V`)** — EMA‑smoothed current voltage (alpha = 0.1)
- **Minimum (`*M`)** — Lowest voltage seen since boot, with slow decay (0.01V/s)
- Both are sent every `BATTERY_REPORT_INTERVAL_MS` (default 2000ms)

---

## Notes

- All commands are ASCII
- Emergency stop overrides all motion and latches until reset with `?`
- Soft stop (`X`) does not latch — also feeds watchdog
- Any valid command feeds watchdog and prevents INPUT_LOSS
- `^` is an explicit keepalive — feeds watchdog without affecting motion
- Watchdog asserts input loss if no valid command arrives within timeout
- Autonomous mode ON (`1`) and OFF (`0`) are stateless — safe to resend
- Autonomous mode exits immediately on any manual input
- HC-05 STATE pin (optional) detects physical disconnection — `CONNECTION_LOSS` state has higher priority than `INPUT_LOSS`