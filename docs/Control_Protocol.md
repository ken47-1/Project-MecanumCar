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
|---|---|
| (none) | Movement / system |
| `%`    | Speed / step mode |

---

## Commands (App → Robot)

### Movement
Single-character, no prefix.

| Command | Action |
|---|---|
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

Multiple keys combine. The parser accumulates every command in one cycle. Holding `W` and `A` together produces the same result as `Q`.

---

### Speed Control (`%` prefix)

| Command | Action |
|---|---|
| `%+` | Increase speed |
| `%-` | Decrease speed |
| `%F` | Fine step mode |
| `%N` | Normal step mode |
| `%R` | Rough step mode |

---

### System

| Command | Action |
|---|---|
| `X` | Soft stop (non-latching) |
| `!` | Emergency stop (latching fault) |
| `?` | Reset fault |
| `1` | Autonomous mode ON |
| `0` | Autonomous mode OFF |
| `T` | Toggle arc turn mode (Fixed ↔ Speed-Dependent) |
| `P` | Toggle closed-loop PID (requires `ENABLE_ENCODERS`) |
| `K` | PID telemetry snapshot (requires `ENABLE_ENCODERS`) |
| `G` + letter | Toggle one log channel (`C I M R P S F W`; `G` reserved for dump) |
| `G` + `G` | Dump full mask |
| `G` + `+` | All channels on |
| `G` + `-` | All channels off |
| `G` + `L` + letter | Toggle one log level (`D I W E`) |

`?` only resets `E-STOP`, `MANUAL`, and user faults. Hardware faults (`SHIELD_NOT_FOUND`, `INTERNAL_ERROR`, `BATTERY_CRITICAL`, `SENSOR_FAIL`) require a power cycle.

The `P` and `K` error paths feed the watchdog too, when encoders are off.

Log commands feed the watchdog.

---

## Feedback (Robot → App)

| Frame | Description |
|---|---|
| `*G[value]*` | Speed gauge — value is `0–1000` |
| `*%[mode]*` | Step mode — `Fine`, `Normal`, or `Rough` |
| `*V[value]V*` | Filtered battery voltage — e.g., `*V7.72V*` |
| `*M[value]V*` | Minimum battery voltage (with decay) — e.g., `*M7.70V*` |

Speed frames re-emit on change. When the value is stable, `*G` and `*%` re-send every `SPEED_FEEDBACK_INTERVAL_MS` (default 2000ms). A reconnecting app recovers the current speed and step mode without waiting for a change.

### Battery Voltage

- **Filtered (`*V`)** — EMA‑smoothed current voltage (alpha = 0.1)
- **Minimum (`*M`)** — Lowest voltage seen since boot, with slow decay (0.01V/s)
- Both are sent every `BATTERY_REPORT_INTERVAL_MS` (default 2000ms)

### Log Output

Debug and Info lines are rate-limited. Warn and Error print immediately and never drop. Intervals are set by `LOG_RATE_LIMIT_D` and `LOG_RATE_LIMIT_I` in `LogConfig.h`.

---

## Notes

- All commands are ASCII
- Emergency stop overrides all motion and latches until reset with `?`
- Soft stop (`X`) does not latch — also feeds watchdog
- Any valid command feeds watchdog and prevents INPUT_LOSS
- On input loss, motors stop within `INPUT_WATCHDOG_TIMEOUT_MS` (default 150 ms). The ramp curve does not finish.
- The app sends `X` at idle as a heartbeat. The parser caps at 64 chars per call.
- Front sensor polling suspends while the servo is off-axis and during a sweep.
- Watchdog asserts input loss if no valid command arrives within `INPUT_WATCHDOG_TIMEOUT_MS` (default 150 ms).
- Autonomous mode ON (`1`) and OFF (`0`) are stateless.
- Autonomous mode exits immediately on any manual input.
- HC-05 STATE pin (optional) detects physical disconnection. See `ENABLE_HC05_STATE_PIN` in HardwareConfig.h. `CONNECTION_LOSS` has higher priority than `INPUT_LOSS`.
- `T` toggles between Fixed (0.5× rotation) and Speed-Dependent (tighter at low speed, wider at high) arc turning. Prints current mode.
- `P` requires `ENABLE_ENCODERS = 1` in `HardwareConfig.h`. When encoders are off, the command prints an error and does nothing. When on, it prints `PID: closed` or `PID: open`.
