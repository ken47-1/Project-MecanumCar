/* ==================== Config.h ==================== */
#pragma once

#include <stdint.h>
#include "HardwareConfig.h"

/* =============== SOFTWARE FEATURES =============== */
/* These control what the firmware does, not what hardware is present. */

/* ============ INPUT FEATURES ============ */
#define ENABLE_INPUT_WATCHDOG        1   // Auto-stop on lost connection
#define ENABLE_INPUT_BUTTONS         1   // WASD/QE/ZC/JL commands
#define ENABLE_INPUT_JOYSTICK        0   // Joystick protocol (future)
#define ENABLE_INPUT_SPEED_AUTHORITY 1   // Speed slider (%+, %-, etc.)

/* ============ NAVIGATION & AUTONOMY ============ */
#define ENABLE_DIRECTIONAL_SCAN     1   // Servo sweep for obstacle avoidance
#define ENABLE_OBSTACLE_AVOIDANCE   1   // ON by default – Requires ultrasonic front/rear
#define ENABLE_AUTONOMOUS_MODE      0   // OFF by default – Requires OBSTACLE_AVOIDANCE + DIRECTIONAL_SCAN

/* =============== SENSORS =============== */

/* ============ ULTRASONIC (FRONT) ============ */
/* Obstacle thresholds (cm)
   Hysteresis: Enter < Exit
   Priority:   STOP < SLOW
*/
constexpr int FRONT_SLOW_ENTER_CM = 30;
constexpr int FRONT_SLOW_EXIT_CM  = 35;
constexpr int FRONT_STOP_ENTER_CM = 15;
constexpr int FRONT_STOP_EXIT_CM  = 20;

static_assert(FRONT_STOP_ENTER_CM < FRONT_SLOW_ENTER_CM, "STOP must be closer than SLOW");
static_assert(FRONT_STOP_EXIT_CM  < FRONT_SLOW_EXIT_CM,  "STOP exit must be closer than SLOW exit");
static_assert(FRONT_SLOW_EXIT_CM  > FRONT_SLOW_ENTER_CM, "SLOW exit must be farther than enter");
static_assert(FRONT_STOP_EXIT_CM  > FRONT_STOP_ENTER_CM, "STOP exit must be farther than enter");

/* ============ ULTRASONIC (REAR) ============ */
/* Obstacle thresholds (cm)
   Hysteresis: Enter < Exit
   Priority:   STOP < SLOW
*/
constexpr int REAR_SLOW_ENTER_CM = 35;
constexpr int REAR_SLOW_EXIT_CM  = 40;
constexpr int REAR_STOP_ENTER_CM = 15;
constexpr int REAR_STOP_EXIT_CM  = 20;

static_assert(REAR_STOP_ENTER_CM < REAR_SLOW_ENTER_CM, "STOP must be closer than SLOW");
static_assert(REAR_STOP_EXIT_CM  < REAR_SLOW_EXIT_CM,  "STOP exit must be closer than SLOW exit");
static_assert(REAR_SLOW_EXIT_CM  > REAR_SLOW_ENTER_CM, "SLOW exit must be farther than enter");
static_assert(REAR_STOP_EXIT_CM  > REAR_STOP_ENTER_CM, "STOP exit must be farther than enter");

/* ============ FILTERING ============ */
/* Exponential Moving Average (EMA): filtered = filtered + a * (raw - filtered)
   Lower a = smoother but slower to react (0.1 = sluggish)
   Higher a = faster response but noisier (0.5 = twitchy)
   Recommended range: 0.3 - 0.5 for a fast-changing Mecanum platform
*/
constexpr float ULTRASONIC_EMA_ALPHA_FRONT = 0.35f;
constexpr float ULTRASONIC_EMA_ALPHA_REAR  = 0.35f;

/* =============== DIRECTIONAL SCAN =============== */
constexpr unsigned long SCAN_SERVO_SETTLE_MS_45  = 200;
constexpr unsigned long SCAN_SERVO_SETTLE_MS_90  = 300;
constexpr unsigned long SCAN_SERVO_SETTLE_MS_135 = 500;

/* =============== BATTERY MONITORING (REV 2) =============== */
/* 0-25V voltage sensor module */

/* --- Voltage Thresholds --- */
constexpr float BATTERY_WARNING_VOLTAGE  = 7.0f;  // Warning below this
constexpr float BATTERY_CRITICAL_VOLTAGE = 6.0f;  // E-stop below this

/* --- Filtering --- */
constexpr float BATTERY_EMA_ALPHA      = 0.1f;   // Smoothing (0.0-1.0)
constexpr float BATTERY_MIN_DECAY_RATE = 0.01f;  // Min voltage recovery (V/s)

/* --- Telemetry --- */
constexpr unsigned long BATTERY_REPORT_INTERVAL_MS = 2000;

/* --- Warning Cooldowns --- */
constexpr unsigned long BATTERY_WARNING_COOLDOWN_MS  = 10000;  // 10s between warnings
constexpr unsigned long BATTERY_CRITICAL_COOLDOWN_MS = 5000;   // 5s between critical alerts

/* =============== INPUT =============== */

/* ============ WATCHDOG ============ */
/* App sends every 50ms ('X' on idle, motion command when active) */
/* 150ms = 3 missed packets before INPUT_LOSS is asserted */
constexpr unsigned long INPUT_WATCHDOG_TIMEOUT_MS = 150;

/* ============ BLUETOOTH PARSER ============ */
/* Max characters to process per loop to prevent blocking */
constexpr uint8_t BT_PARSER_MAX_CHARS = 50;

/* ============ JOYSTICK ============ */
// Unused (to be implemented later)
constexpr float JOYSTICK_DEADZONE  = 30.0f;
constexpr float JOYSTICK_INPUT_MAX = 127.0f;

/* =============== CONTROL =============== */

/* ============ ARC TURNING ============ */
/*
   Default mode on boot. Toggle at runtime with 'T' command.
   0 = Fixed (predictable, always 0.5x rotation)
   1 = Speed-Dependent (tighter at low speed, wider at high)
*/
#define ARC_TURN_DEFAULT_MODE 1

/* Fixed mode: multiplier applied to rotation when moving forward/backward */
constexpr float FIXED_ROTATE_SCALE = 0.5f;

/* Speed-Dependent mode: min/max scale range */
constexpr float SD_MIN_SCALE = 0.3f;   // At full speed
constexpr float SD_MAX_SCALE = 0.7f;   // At low speed

/* ============ OBSTACLE AVOIDANCE ============ */
constexpr unsigned long OA_ACTION_HOLD_MS     = 500;    // How long to hold action (ms)
constexpr unsigned long OA_CLEAR_HOLD_MS      = 200;    // hold before clearing a zone flag
constexpr float         OA_SOFT_AUTHORITY     = 0.5f;   // speed scale in slow zone

/* ============ AUTONOMOUS MODE ============ */
/* Max speed cap in autonomous mode (per-mille, 0-1000) */
constexpr uint16_t AUTO_SPEED = 600;

/* How long to wait before retrying when all directions are blocked */
constexpr unsigned long AUTO_RETRY_WAIT_MS = 2000;

/* Time required to rotate the chassis (milliseconds) */
/* ONLY USED IF #define ENABLE_ENCODERS IS DISABLED */
constexpr uint16_t AUTO_SPIN_DIAGONAL_MS = 500;   // For 45-degree adjustments
constexpr uint16_t AUTO_SPIN_SIDE_MS     = 1000;  // For 90-degree adjustments

/* ============ DRIVE BEHAVIOR ============ */

/* ------ Speed Authority ------ */
/*
    Speed is a scalar applied to normalized intent.
    Units are per-mille (0-1000), converted to float (0.0-1.0) internally.
*/
constexpr uint16_t SPEED_USER_MIN     = 200;
constexpr uint16_t SPEED_USER_MAX     = 1000;
constexpr uint16_t SPEED_USER_DEFAULT = 1000;

/* Below this speed, soft OA scaling is bypassed (user is intentionally creeping) */
constexpr uint16_t SPEED_AUTHORITY_THRESHOLD_USER = 600;

/* Step resolution */
constexpr uint16_t SPEED_STEP_ROUGH  = 100;  // 10.0%
constexpr uint16_t SPEED_STEP_NORMAL = 50;   // 5.0%
constexpr uint16_t SPEED_STEP_FINE   = 10;   // 1.0%

/* ------ Speed Ramp ------ */
/*
    Time to ramp from 0 -> 100% command (or 100% -> 0).
    Independent of loop frequency - always completes in the given time.

    100 ms  = very fast / aggressive
    200 ms  = quick but controllable
    300 ms  = smooth (recommended)
    500 ms  = soft / gentle
    >800 ms = sluggish
*/
constexpr uint16_t RAMP_UP_TIME_MS   = 400;
constexpr uint16_t RAMP_DOWN_TIME_MS = 200;

/* ------ Turn Rate ------ */
constexpr int16_t TURN_RATIO_NUM = 1;
constexpr int16_t TURN_RATIO_DEN = 2;
static_assert(TURN_RATIO_DEN > 0, "TURN_RATIO_DEN must be > 0");

/* =============== HARDWARE =============== */

/* ============ MOTOR OUTPUT ============ */
/* PWM resolution - change when switching motor drivers */
constexpr uint16_t PWM_MAX = 4095;     // AFMS V2 (12-bit)
// constexpr uint16_t PWM_MAX = 255;   // AFMS V1 (8-bit)

/* ============ ENCODERS (REV 3) ============ */
constexpr uint16_t ENCODER_TICKS_PER_REV = 20;

/* ============ PID ============ */
constexpr float PID_KP = 1.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;

/* =============== DEPENDENCY CHECKS =============== */

#if ENABLE_OBSTACLE_AVOIDANCE && !ENABLE_ULTRASONIC_FRONT
    #error "Obstacle avoidance requires front ultrasonic sensor (ENABLE_ULTRASONIC_FRONT)"
#endif

#if ENABLE_DIRECTIONAL_SCAN && !ENABLE_SERVO
    #error "Directional scan requires servo (ENABLE_SERVO)"
#endif

#if ENABLE_DIRECTIONAL_SCAN && !ENABLE_ULTRASONIC_FRONT
    #error "Directional scan requires front ultrasonic sensor (ENABLE_ULTRASONIC_FRONT)"
#endif

#if ENABLE_AUTONOMOUS_MODE && !ENABLE_OBSTACLE_AVOIDANCE
    #error "Autonomous mode requires obstacle avoidance (ENABLE_OBSTACLE_AVOIDANCE)"
#endif

#if ENABLE_AUTONOMOUS_MODE && !ENABLE_DIRECTIONAL_SCAN
    #error "Autonomous mode requires directional scan (ENABLE_DIRECTIONAL_SCAN)"
#endif