/* ==================== Config.h ==================== */
#pragma once

#include <stdint.h>

/* =============== DEBUG =============== */
#define DEBUG_ENABLED  1   // Master toggle

#if DEBUG_ENABLED   // EDIT BELOW
  #define COMMS_DEBUG_MIRROR  1
  #define DEBUG_COMMS         0
  #define DEBUG_WATCHDOG      0
  #define DEBUG_SENSORS       0
  #define DEBUG_MOTOR_RAMP    0
  #define DEBUG_OA_REASON     0
  #define DEBUG_OA_SCALE      0
#else   // DO NOT EDIT BELOW
  #define COMMS_DEBUG_MIRROR  0
  #define DEBUG_COMMS         0
  #define DEBUG_WATCHDOG      0
  #define DEBUG_SENSORS       0
  #define DEBUG_MOTOR_RAMP    0
  #define DEBUG_OA_REASON     0
  #define DEBUG_OA_SCALE      0
#endif

/* =============== HARDWARE CONFIG =============== */

/* ============ SERVO ============ */
/* Pins */
constexpr uint8_t SCAN_SERVO_PIN = 10;  // AFMS V2 CLONE Servo 1 (D10)

/* Angles (degrees) */
constexpr int SERVO_LEFT        = 180;
constexpr int SERVO_FRONT_LEFT  = 135;
constexpr int SERVO_CENTER      = 90;
constexpr int SERVO_FRONT_RIGHT = 45;
constexpr int SERVO_RIGHT       = 0;

/* ============ ULTRASONIC SENSORS ============ */

/* ========= FRONT ========= */
/* Pins */
constexpr uint8_t SR04_FRONT_TRIG_PIN = 8;
constexpr uint8_t SR04_FRONT_ECHO_PIN = 9;

/* Thresholds (cm) — enter < exit, stop < slow */
constexpr int FRONT_SLOW_ENTER_CM = 40;
constexpr int FRONT_SLOW_EXIT_CM  = 50;
constexpr int FRONT_STOP_ENTER_CM = 15;
constexpr int FRONT_STOP_EXIT_CM  = 25;

static_assert(FRONT_STOP_ENTER_CM < FRONT_SLOW_ENTER_CM, "STOP must be closer than SLOW");
static_assert(FRONT_STOP_EXIT_CM  < FRONT_SLOW_EXIT_CM,  "STOP exit must be closer than SLOW exit");
static_assert(FRONT_SLOW_EXIT_CM  > FRONT_SLOW_ENTER_CM, "SLOW exit must be farther than enter");
static_assert(FRONT_STOP_EXIT_CM  > FRONT_STOP_ENTER_CM, "STOP exit must be farther than enter");

/* ========= REAR ========= */
/* Pins */
constexpr uint8_t SR04_REAR_TRIG_PIN = 6;
constexpr uint8_t SR04_REAR_ECHO_PIN = 7;

/* Thresholds (cm) — enter < exit, stop < slow */
constexpr int REAR_SLOW_ENTER_CM = 40;
constexpr int REAR_SLOW_EXIT_CM  = 50;
constexpr int REAR_STOP_ENTER_CM = 15;
constexpr int REAR_STOP_EXIT_CM  = 25;

static_assert(REAR_STOP_ENTER_CM < REAR_SLOW_ENTER_CM, "STOP must be closer than SLOW");
static_assert(REAR_STOP_EXIT_CM  < REAR_SLOW_EXIT_CM,  "STOP exit must be closer than SLOW exit");
static_assert(REAR_SLOW_EXIT_CM  > REAR_SLOW_ENTER_CM, "SLOW exit must be farther than enter");
static_assert(REAR_STOP_EXIT_CM  > REAR_STOP_ENTER_CM, "STOP exit must be farther than enter");

/* ========= SENSOR FILTERING ========= */
/* Exponential Moving Average (EMA): filtered = filtered + a * (raw - filtered) */
/* Lower a = smoother but slower to react (0.1 = sluggish) */
/* Higher a = faster response but noisier (0.5 = twitchy) */
/* Recommended range: 0.3 - 0.5 for a fast-changing Mecanum platform */
constexpr float ULTRASONIC_EMA_ALPHA_FRONT = 0.35f;
constexpr float ULTRASONIC_EMA_ALPHA_REAR  = 0.35f;

/* ============ BLUETOOTH ============ */
/* Bluetooth: R3 uses Serial (pins 0/1), R4 uses Serial1 (pins 0/1) */
/* R3: Disconnect Bluetooth when uploading (pins shared with USB) */
/* R4: Upload with Bluetooth connected (Serial1 is independent) */

/* Set to 1 for HC-05 (configurable STATE pin), set to 0 for HC-06 */
#define ENABLE_HC05_STATE_PIN  0
#if ENABLE_HC05_STATE_PIN
    constexpr uint8_t BT_STATE_PIN = 2;  // HC-05 STATE pin
#endif

/* =============== FEATURE FLAGS (COMPILE-TIME) =============== */

/* ============ FEATURES ============ */
#define ENABLE_INPUT_WATCHDOG      1
#define ENABLE_OBSTACLE_AVOIDANCE  0

/* ============ INPUT MODES ============ */
#define ENABLE_INPUT_BUTTONS          1
#define ENABLE_INPUT_JOYSTICK         0
#define ENABLE_INPUT_SPEED_AUTHORITY  1

/* =============== FEATURE CONFIG =============== */

/* ============ INPUT WATCHDOG ============ */
/* App sends every 50ms ('X' on idle, motion command when active) */
/* 150ms = 3 missed packets before INPUT_LOSS is asserted */
constexpr unsigned long INPUT_WATCHDOG_TIMEOUT_MS = 150;

/* ============ JOYSTICK ============ */
constexpr float JOYSTICK_DEADZONE  = 30.0f;    // raw units
constexpr float JOYSTICK_INPUT_MAX = 127.0f;   // raw signed max

/* ============ DIRECTIONAL SCAN ============ */
/* 500ms matches the proven settle time from the reference implementation */
constexpr unsigned long SCAN_SERVO_SETTLE_MS = 500;

/* ============ OBSTACLE AVOIDANCE ============ */
constexpr unsigned long OA_CLEAR_HOLD_MS  = 200;    // hold before clearing a zone flag
constexpr float         OA_SOFT_AUTHORITY = 0.5f;   // speed scale in slow zone
constexpr float         OA_BACKOFF_SPEED  = 0.25f;  // speed for nudge-away backoff

/* ============ AUTONOMOUS MODE ============ */
/* Max speed cap in autonomous mode (per-mille, 0-1000) */
constexpr uint16_t AUTO_SPEED = 600;

/* How long to wait before retrying when all directions are blocked */
constexpr unsigned long AUTO_RETRY_WAIT_MS = 2000;

/* Time required to rotate the chassis (milliseconds) */
constexpr uint16_t AUTO_SPIN_DIAGONAL_MS = 500;   // For 45-degree adjustments
constexpr uint16_t AUTO_SPIN_SIDE_MS     = 1000;  // For 90-degree adjustments

/* =============== DRIVE BEHAVIOR =============== */

/* ============ TURN RATE ============ */
constexpr int16_t TURN_RATIO_NUM = 1;
constexpr int16_t TURN_RATIO_DEN = 2;

static_assert(TURN_RATIO_DEN > 0, "TURN_RATIO_DEN must be > 0");

/* =============== SPEED AUTHORITY =============== */
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
constexpr uint16_t SPEED_STEP_ROUGH  = 100;   // 10.0%
constexpr uint16_t SPEED_STEP_NORMAL = 50;    // 5.0%
constexpr uint16_t SPEED_STEP_FINE   = 10;    // 1.0%

/* =============== MOTOR OUTPUT =============== */
/* PWM resolution - change when switching motor drivers */
constexpr uint16_t PWM_MAX = 4095;     // AFMS V2 (12-bit)
// constexpr uint16_t PWM_MAX = 255;   // AFMS V1 (8-bit)

/* =============== SPEED RAMP =============== */
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
