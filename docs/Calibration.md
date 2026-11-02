# Calibration Guide

The robot calculates odometry and closed-loop PID using physical measurements. Measure your specific hardware before you enable autonomous mode or closed-loop PID.

## 1. Calibrate Encoders

Measure the encoder ticks for one full wheel revolution.

1. Lift the robot so the wheels spin freely.
2. Mark one wheel with tape.
3. Flash a test sketch that prints `Encoder::get_count(0)` to the serial monitor.
4. Reset the board.
5. Spin the marked wheel by hand exactly one revolution.
6. Read the count.
7. Set `ENCODER_TICKS_PER_REV` in `Config.h` to the measured count.

## 2. Calibrate Kinematics

Measure the physical dimensions of the chassis. The odometry module uses these values.

1. Use calipers to measure the exact diameter of one wheel in millimeters.
2. Set `WHEEL_DIAMETER_MM` in `Config.h` to this value. Do not use the manufacturer specification.
3. Measure the distance from the center of the left wheels to the center of the right wheels.
4. Set `TRACK_WIDTH_MM` in `Config.h` to this value.

## 3. Calibrate Motor Limits

Measure the maximum speed of each motor.

1. Place the robot on the floor with 3 meters of clear space.
2. Open a terminal and run `tools/drive.py`.
3. Enable the PID log channel (`G` then `P`).
4. Drive the robot forward at 100% speed for 3 seconds.
5. Read the `[D][PID ] FL=... FR=... RL=... RR=...` output.
6. Note the steady-state RPM for each wheel.
7. Set `MOTOR_MAX_RPM_FL`, `MOTOR_MAX_RPM_FR`, `MOTOR_MAX_RPM_RL`, and `MOTOR_MAX_RPM_RR` in `Config.h` to the measured values.

## 4. Calibrate PID

Tune the PID to maintain accurate speeds under load.

### 4.1. Find Deadband

1. Open `tools/drive.py` and hold `W`. The robot starts at 100% speed.
2. Press `%-` to step down the speed gradually.
3. Note the exact percentage where the wheels stop spinning.
4. Set `PID_MIN_OUTPUT` in `Config.h` to this value (for example, `0.10f`).

### 4.2. Tune Gains

1. Set `PID_KP` to `1.5f`, `PID_KI` to `0.0f`, and `PID_KD` to `0.0f`.
2. Enable closed-loop mode (`P`).
3. Drive forward and backward.
4. If the robot jerks or oscillates, decrease `PID_KP`.
5. If the robot feels sluggish, increase `PID_KP`.
6. If one side lags behind, increase `PID_KI` slightly (for example, `0.05f`).

## 5. Calibrate Battery

Update the limits if you use a different battery chemistry (for example, 3S LiPo instead of 2S Li-ion).

1. Check the resistor values on your voltage sensor module.
2. Calculate the divider ratio: `(R1 + R2) / R2`.
3. Set `BATTERY_DIVIDER_RATIO` in `HardwareConfig.h`.
4. Set `BATTERY_VOLTAGE_MIN` and `BATTERY_VOLTAGE_MAX` to the safe limits of your pack.
5. Set `BATTERY_CRITICAL_VOLTAGE` and `BATTERY_WARNING_VOLTAGE` in `Config.h` to your alert thresholds.