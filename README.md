# Project MecanumCar

*This project was developed with AI-assisted code generation and human oversight.*

Bluetooth-controlled Mecanum wheel robot with autonomous obstacle avoidance. The robot uses an Arduino Uno and an Adafruit Motor Shield V2. The firmware uses modular C++ on the Arduino Framework. It includes dual HC-SR04 ultrasonic sensors with a servo turret and a real-time safety subsystem.

## Hardware

- **MCU:** Arduino Uno R3 (ATmega328P) or Uno R4 (Minima/WiFi).
- **Motor Driver:** Adafruit Motor Shield V2 (TB6612FNG + PCA9685, 12-bit PWM).
- **Wheels:** 4x mecanum (45° rubber rollers).
- **Sensors:** HC-SR04 front (servo-mounted, 5-position sweep) and HC-SR04 rear.
- **Servo:** Micro servo (SG90 or compatible).
- **Bluetooth:** HC-05 or HC-06 (9600 baud).
- **Power:** 2x 18650 lithium cells in series.
- **Chassis:** Clear acrylic frame.

### Bluetooth Module Support

| Module | STATE Pin | Connection Detection |
|---|---|---|
| HC-05 | Yes (D2) | Detects Bluetooth disconnection (disabled by default) |
| HC-06 | No | Relies on watchdog timeout only |

**HC-05 users:** Connect the STATE pin to D2. Set `ENABLE_HC05_STATE_PIN = 1` in `HardwareConfig.h`.
**HC-06 users:** Leave `ENABLE_HC05_STATE_PIN = 0` in `HardwareConfig.h`.

### Pin Configuration

| Component | Pins |
|---|---|
| Front Ultrasonic TRIG/ECHO | D11 / D12 |
| Rear Ultrasonic TRIG/ECHO | D8 / D9 |
| Servo | D10 |
| Bluetooth RX/TX | D0 / D1 |
| Bluetooth STATE (HC-05 only) | D2 |
| Battery Monitor | A0 |
| Encoders | D7, D6, D5, D4 |

**Note:** On R3, disconnect Bluetooth when uploading. On R4, upload with Bluetooth connected.

### Motor Mapping (AFMS V2)

| Wheel | Motor Port |
|---|---|
| Front Left | M1 |
| Front Right | M4 |
| Rear Left | M2 |
| Rear Right | M3 |

## Quick Start

### 1. Flash

```bash
# Arduino Uno R3
pio run -e uno -t upload

# Arduino Uno R4 Minima/WiFi
pio run -e uno_r4_minima -t upload

# Monitor
pio device monitor -b 9600
```

### 2. Wiring

Connect Bluetooth to pins D0 (RX) and D1 (TX).
- **HC-05:** Connect the STATE pin to D2.
- **HC-06:** No STATE pin.

### 3. Pair Bluetooth

1. Open phone Bluetooth settings.
2. Pair with HC-05 or HC-06 (PIN is usually 1234 or 0000).
3. Install the Bluetooth Electronics app on Android.
4. Load the custom panel from `archive/Bluetooth Electronics Panels/`.

### 4. Drive

- **Manual Mode (`0`):** Use the d-pad to move. Use the speed slider to control speed.
- **Autonomous Mode (`1`):** The robot navigates obstacles automatically.

## Configuration

Settings are split across three files in `include/config/`.

- `HardwareConfig.h`: Physical hardware presence.
- `Config.h`: Software behavior.
- `LogConfig.h`: Compile-time log gate.

### Features

**Input** (`Config.h`)
- `ENABLE_INPUT_WATCHDOG`: Bluetooth keepalive.
- `ENABLE_INPUT_BUTTONS`: WASD/QE/ZC/JL commands.
- `ENABLE_INPUT_SPEED_AUTHORITY`: Speed slider.

**Navigation & Autonomy** (`Config.h`)
- `ENABLE_DIRECTIONAL_SCAN`: Servo sweep.
- `ENABLE_OBSTACLE_AVOIDANCE`: Front/rear veto logic.
- `ENABLE_AUTONOMOUS_MODE`: State machine. Requires `ENABLE_DIRECTIONAL_SCAN`.

**Hardware Presence** (`HardwareConfig.h`)
- `ENABLE_HC05_STATE_PIN`: HC-05 STATE pin.
- `ENABLE_ULTRASONIC_FRONT`: Front HC-SR04.
- `ENABLE_ULTRASONIC_REAR`: Rear HC-SR04.
- `ENABLE_SERVO`: Servo for directional scan.
- `ENABLE_BATTERY_MONITOR`: 0-25V voltage sensor.
- `ENABLE_ENCODERS`: H206 optical encoders.

### Drive Behavior

- **Speed:** `SPEED_USER_MIN` (200), `SPEED_USER_MAX` (1000), `SPEED_USER_DEFAULT` (1000).
- **Speed steps:** `SPEED_STEP_ROUGH` (10%), `SPEED_STEP_NORMAL` (5%), `SPEED_STEP_FINE` (1%).
- **Motor ramp:** `RAMP_UP_TIME_MS` (400 ms), `RAMP_DOWN_TIME_MS` (200 ms).
- **Autonomous speed:** `AUTO_SPEED` (600).

### Obstacle Avoidance

- **Autonomous mode:** Bypasses the user speed slider.
- **Front slow:** 30 to 35 cm (0.5x speed).
- **Front stop:** 15 to 20 cm (clamps forward motion).
- **Rear slow:** 35 to 40 cm (0.5x speed).
- **Rear stop:** 15 to 20 cm (clamps reverse motion).
- **Hold time:** `OA_CLEAR_HOLD_MS` (200 ms).

### Autonomous Mode

- **Retry wait:** `AUTO_RETRY_WAIT_MS` (2000 ms).
- **Spin time:** `AUTO_SPIN_DIAGONAL_MS` (500 ms), `AUTO_SPIN_SIDE_MS` (1000 ms).
- **Servo settle:** 45°=200 ms, 90°=300 ms, 135°=500 ms.

## Controls

### Movement

- `W/S`: Forward / Backward.
- `A/D`: Strafe left / right.
- `Q/E/Z/C`: Diagonals.
- `J/L`: Rotate CCW / CW.
- `X`: Stop / Heartbeat.

### Speed

- `%+` / `%-`: Increase / Decrease speed.
- `%R`: Rough (10% steps).
- `%N`: Normal (5% steps).
- `%F`: Fine (1% steps).

### System

- `!`: Emergency stop (latches).
- `?`: Reset / clear E-stop and SENSOR_FAIL.
- `0`: Manual mode.
- `1`: Autonomous mode.
- `T`: Arc turn toggle.
- `P`: Toggle closed-loop PID.
- `G` + letter: Toggle log channel.
- `G` + `G`: Dump full mask.
- `G` + `+` / `G` + `-`: All channels on / off.
- `G` + `L` + letter: Toggle log level.

## Safety Features

- **Watchdog:** 150 ms Bluetooth timeout. Armed only while moving. Any valid command resets the timer. If no command arrives, INPUT_LOSS is asserted and the motors stop.
- **Obstacle Detection:** Front/rear independent.
  - Clear: >35 cm front, >40 cm rear.
  - Slow: 30 to 35 cm front, 35 to 40 cm rear.
  - Stop: 15 to 20 cm.
  - Sensor dropout (`dist==0`) is treated as clear.
  - Front blocked allows reverse. Rear blocked allows forward.

## Autonomous Mode

1. Drives forward at `AUTO_SPEED` (600).
2. On obstacle: stops and sweeps the servo.
3. Picks the clearest path and rotates toward it (using encoder ticks if enabled).
4. Resumes driving.
5. If cornered: waits `AUTO_RETRY_WAIT_MS` and retries.
6. If `STUCK_MAX_RETRIES` is reached, holds position.

## Architecture & Code

**Modules** (`src/`):
- `comms/`: Bluetooth parser, serial output.
- `control/`: Motor control, ramping, autonomous state machine.
- `input/`: Watchdog, button parsing, speed authority.
- `safety/`: Obstacle detection, motion policy.
- `sensors/`: HC-SR04 wrapper, servo sweep, battery monitor, encoders, odometry.

**Design Patterns**:
- Non-blocking architecture.
- Threshold tuning over math compensation.
- Emergent watchdog.
- EMA filtering for sensor noise.

## Troubleshooting

**Robot won't move**
- Check E-stop: send `?` to reset.
- Verify Bluetooth connected.
- Check battery voltage (>6V).
- Check SafetyManager state.

**Bluetooth keeps disconnecting**
- Move closer to the phone.
- Reduce electromagnetic interference.
- Re-pair.
- HC-05 users: verify `ENABLE_HC05_STATE_PIN`.

**Obstacle avoidance not working**
- Check sensor wires.
- Watch serial output: `[D][SNR ] FRONT=XX REAR=XX`.
- Adjust thresholds in `Config.h`.

**Servo doesn't scan**
- Check D10 connection.
- Verify servo power.
- Adjust `SERVO_LEFT`, `SERVO_CENTER`, `SERVO_RIGHT`.

**Encoders read 0 on R4**
- Ensure `AGTimer_R4_Library` is installed in your build environment.
- Verify the encoder wires connect to D4-D7.
- Check the encoder power connections.

**Robot stops and shows CONNECTION_LOSS**
- HC-05 only: STATE pin detected disconnection.
- Re-pair Bluetooth or restart the app.

**R4 serial monitor is blank**
- The R4 USB CDC port requires the host to connect. Press the reset button on the board while the monitor is open.

## Performance

| Metric | Value |
|---|---|
| Command latency | <10 ms |
| Motor response | <50 ms |
| Ultrasonic sampling | ~50 ms per sensor |
| Servo sweep | ~1.2 seconds |
| Speed ramp | 400 ms accel, 200 ms decel |
| Watchdog timeout | 150 ms |
| Max speed | ~1.5 m/s |

## For Developers

This project enforces a strict code layout standard in [`docs/Code_Layout_Standard.md`](docs/Code_Layout_Standard.md).

- One logical module per file.
- Headers declare public API only.
- Source files contain all implementation and internal state.
- Comment hierarchy: T1 (file header) to T7 (inline notes).

## License

MIT
