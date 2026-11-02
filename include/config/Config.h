/* ==================== Config.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "HardwareConfig.h"

/* ============ CORE ============ */
#include <stdint.h>

/* =============== SOFTWARE FEATURES =============== */
/* These control what the firmware does, not what hardware is present. */

/* ============ INPUT ============ */
#define ENABLE_INPUT_WATCHDOG        1   // Auto-stop on lost connection
#define ENABLE_INPUT_BUTTONS         1   // WASD/QE/ZC/JL commands
#define ENABLE_INPUT_SPEED_AUTHORITY 1   // Speed slider (%+, %-, etc.)

/* ============ NAVIGATION ============ */
#define ENABLE_DIRECTIONAL_SCAN   1   // Servo sweep for obstacle avoidance
#define ENABLE_OBSTACLE_AVOIDANCE 1   // ON by default – Requires ultrasonic front/rear
#define ENABLE_AUTONOMOUS_MODE    0   // OFF by default – Requires OBSTACLE_AVOIDANCE + DIRECTIONAL_SCAN

/* =============== COMMS =============== */
constexpr unsigned long BAUD_RATE         = 9600;
constexpr unsigned long USB_CDC_WAIT_MS   = 2000; // R4 boot wait for host
constexpr uint8_t       RX_DRAIN_MAX      = 64;   // Max chars parsed per loop

/* =============== INPUT =============== */

/* ============ WATCHDOG ============ */
/* App sends every 50ms ('X' on idle, motion command when active) */
/* 150ms = 3 missed packets before INPUT_LOSS is asserted */
constexpr unsigned long INPUT_WATCHDOG_TIMEOUT_MS = 150;

/* ============ SPEED FEEDBACK ============ */
/* Re-send *G (gauge) and *% (step mode) periodically, not only on change. */
constexpr unsigned long SPEED_FEEDBACK_INTERVAL_MS = 2000;

/* ============ SPEED AUTHORITY ============ */
/* Speed is a scalar applied to normalized intent. Units are per-mille (0-1000). */
constexpr uint16_t SPEED_USER_MIN     = 200;
constexpr uint16_t SPEED_USER_MAX     = 1000;
constexpr uint16_t SPEED_USER_DEFAULT = 1000;

/* Below this speed, soft OA scaling is bypassed (user is intentionally creeping) */
constexpr uint16_t SPEED_AUTHORITY_THRESHOLD_USER = 600;

/* Step resolution */
constexpr uint16_t SPEED_STEP_ROUGH  = 100;  // 10.0%
constexpr uint16_t SPEED_STEP_NORMAL = 50;   // 5.0%
constexpr uint16_t SPEED_STEP_FINE   = 10;   // 1.0%

static_assert(SPEED_USER_MIN <= SPEED_USER_DEFAULT && SPEED_USER_DEFAULT <= SPEED_USER_MAX, 
              "Default speed must be within min/max bounds");

/* =============== CONTROL & DRIVE =============== */

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
constexpr uint32_t RAMP_DT_MAX_MS    = 100;  // Max dt for ramp updates

/* ------ Arc Turning ------ */
/*
   Default mode on boot. Toggle at runtime with 'T' command.
   0 = Fixed (predictable, always 0.5x rotation)
   1 = Speed-Dependent (tighter at low speed, wider at high)
*/
#define ARC_TURN_DEFAULT_MODE 1

constexpr float FIXED_ROTATE_SCALE = 0.5f;
constexpr float SD_MIN_SCALE       = 0.3f;   // At full speed
constexpr float SD_MAX_SCALE       = 0.7f;   // At low speed

static_assert(SD_MIN_SCALE < SD_MAX_SCALE, "SD_MIN_SCALE must be smaller than SD_MAX_SCALE");

/* =============== KINEMATICS =============== */
/* These values are required for odometry and encoder-based autonomous turns. 
   If they are wrong, the robot will not drive straight or turn accurately. */

/* Measure the wheel with calipers. Do not trust the spec sheet. */
constexpr float WHEEL_DIAMETER_MM = 60.0f; 

/* Measure from the center of the left wheels to the center of the right wheels. */
constexpr float TRACK_WIDTH_MM    = 150.0f; 

/* =============== MOTORS & PID =============== */

/* ============ MOTOR OUTPUT ============ */
constexpr uint16_t PWM_MAX           = 4095;   // AFMS V2 (12-bit)
// constexpr uint16_t PWM_MAX           = 255;    // AFMS V1 (8-bit)
constexpr float MOTOR_SIGN_DEADBAND  = 0.01f;  // For direction state
constexpr float MOTOR_STOP_THRESHOLD = 0.05f;  // Ramp stop threshold

/* ============ PER-WHEEL SPEED CEILING ============ */
/* Placeholder 255.0f. Measure exact physical limits with tools/drive.py. */
constexpr float MOTOR_MAX_RPM_FL = 255.0f;
constexpr float MOTOR_MAX_RPM_FR = 255.0f;
constexpr float MOTOR_MAX_RPM_RL = 255.0f;
constexpr float MOTOR_MAX_RPM_RR = 255.0f;

/* ============ PID GAINS ============ */
/* Default 0.0f = open-loop passthrough. 
   Users must tune these after calibrating MOTOR_MAX_RPM. */
constexpr float PID_KP[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
constexpr float PID_KI[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
constexpr float PID_KD[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

constexpr float    PID_INTEGRAL_LIMIT = 0.5f;
constexpr uint16_t PID_PERIOD_MS      = 20;
constexpr float    PID_DT_MAX         = 0.05f;

/* Minimum output floor (motor deadband). 0.0f = disabled.
   If motors stall at low PID outputs, use drive.py to find the minimum PWM 
   required to move the wheels from a standstill, and set it here (e.g., 0.10f). */
constexpr float    PID_MIN_OUTPUT     = 0.0f;

/* ============ ENCODERS ============ */
/* How to calibrate: Mark a wheel with tape, reset the board, 
   spin the wheel exactly 1 revolution by hand, and read Encoder::get_count(). */
constexpr uint16_t ENCODER_TICKS_PER_REV = 20;
constexpr uint32_t ENCODER_DEBOUNCE_US   = 50;
constexpr uint16_t ENCODER_POLL_PERIOD_US= 250;  // AGTimer 4kHz (R4)

/* ------ Encoder Stall Detection ------ */
/* Time since last pulse before RPM is forced to 0.
   Assuming 20 ticks/rev: 100ms=30RPM, 250ms=12RPM, 500ms=6RPM. */
constexpr uint32_t ENCODER_STALL_TIMEOUT_US = 500000;  // 500 ms
constexpr float    PID_STALL_INTENT_MIN     = 0.3f;
constexpr uint16_t PID_STALL_TICKS          = 25;

static_assert(ENCODER_POLL_PERIOD_US < ENCODER_STALL_TIMEOUT_US, 
              "Encoder poll rate must be faster than stall timeout");
static_assert(PID_PERIOD_MS * 1000 < ENCODER_STALL_TIMEOUT_US, 
              "PID period must be faster than stall timeout");

/* ------ Mode ------ */
/* 1 = closed loop (PID active), 0 = open loop.
   Runtime toggle with the 'P' command. */
#define PID_CLOSED_LOOP_DEFAULT 0

/* =============== SAFETY & SENSORS =============== */

/* Fallback distance (cm) used if a sensor times out or disconnects.
   Assumes the path is clear to prevent false stops on sensor failure. */
constexpr uint16_t DISTANCE_FALLBACK_CM = 999; 

/* ============ OBSTACLE AVOIDANCE ============ */
/* How long a zone stays active after the obstacle clears (ms). 
   Prevents rapid toggling at the threshold boundary. */
constexpr unsigned long OA_CLEAR_HOLD_MS  = 200;    

/* Speed multiplier (0.0 to 1.0) applied when driving through a slow zone. */
constexpr float         OA_SOFT_AUTHORITY = 0.5f;   

/* --- Ultrasonic Thresholds --- */
/* Tune these based on your robot's physical size and stopping distance. 
   Increase these values if the robot bumps into walls before stopping. */
constexpr int FRONT_SLOW_ENTER_CM = 30;
constexpr int FRONT_SLOW_EXIT_CM  = 35;
constexpr int FRONT_STOP_ENTER_CM = 15;
constexpr int FRONT_STOP_EXIT_CM  = 20;

constexpr int REAR_SLOW_ENTER_CM  = 35;
constexpr int REAR_SLOW_EXIT_CM   = 40;
constexpr int REAR_STOP_ENTER_CM  = 15;
constexpr int REAR_STOP_EXIT_CM   = 20;

static_assert(FRONT_STOP_ENTER_CM < FRONT_SLOW_ENTER_CM, "STOP must be closer than SLOW");
static_assert(REAR_STOP_ENTER_CM  < REAR_SLOW_ENTER_CM,  "STOP must be closer than SLOW");

/* ============ ULTRASONIC ============ */
/* Max range (cm) to wait for an echo. Lower = faster loop times, but less range. */
constexpr uint16_t ULTRASONIC_MAX_DIST_CM = 90;  
/* Readings above this (cm) are discarded as noise/ghosts. */
constexpr uint16_t ULTRASONIC_INVALID_CM  = 400; 

/* EMA Filtering: Lower alpha = smoother but slower to react. 
   Higher alpha = faster response but noisier. (Recommended: 0.3 - 0.5) */
constexpr float ULTRASONIC_EMA_ALPHA_FRONT = 0.35f;
constexpr float ULTRASONIC_EMA_ALPHA_REAR  = 0.35f;

/* ============ DIRECTIONAL SCAN ============ */
/* Wait times (ms) for the servo to physically reach the angle before pinging.
   Increase if your servo is slow or heavy, decrease to speed up the scan. */
constexpr unsigned long SCAN_SERVO_SETTLE_MS_45  = 200;
constexpr unsigned long SCAN_SERVO_SETTLE_MS_90  = 300;
constexpr unsigned long SCAN_SERVO_SETTLE_MS_135 = 500;

/* ============ BATTERY MONITORING ============ */
/* 10-bit Arduino ADC = 1023. Logic level = 5.0V. */
constexpr float ADC_RESOLUTION  = 1023.0f;
constexpr float ADC_REF_VOLTAGE = 5.0f;

/* Change these to match your battery chemistry. 
   (Defaults are for a 2S Li-Ion pack). 3S LiPo is ~9.0V - 12.6V. */
constexpr float BATTERY_WARNING_VOLTAGE  = 7.0f;
constexpr float BATTERY_CRITICAL_VOLTAGE = 6.0f;
constexpr float BATTERY_CRITICAL_BUFFER  = 0.3f; 

/* EMA smoothing for voltage. Lower = smoother but slower to react. */
constexpr float BATTERY_EMA_ALPHA      = 0.1f;   
/* How fast the tracked minimum voltage recovers (V/s) when unloaded. */
constexpr float BATTERY_MIN_DECAY_RATE = 0.01f;  

/* How often to send battery telemetry (*V and *M frames). */
constexpr unsigned long BATTERY_REPORT_INTERVAL_MS   = 2000;
/* Cooldowns to prevent spamming the app with warnings. */
constexpr unsigned long BATTERY_WARNING_COOLDOWN_MS  = 10000;
constexpr unsigned long BATTERY_CRITICAL_COOLDOWN_MS = 5000;

/* =============== AUTONOMY =============== */

/* Forward speed cap in autonomous mode (per-mille). 600 = 60%. */
constexpr uint16_t AUTO_SPEED         = 600;  
/* Escape/reverse speed cap in autonomous mode. */
constexpr uint16_t AUTO_REVERSE_SPEED = 300;  

/* How long to reverse when escaping a collision (ms). */
constexpr unsigned long AUTO_BACKUP_MS      = 500;
/* How long to wait when completely stuck before trying again. */
constexpr unsigned long AUTO_RETRY_WAIT_MS  = 2000;
/* Max attempts to escape before giving up and stopping. */
constexpr uint8_t       STUCK_MAX_RETRIES   = 5;

/* Fallback spin times (ms) if encoders are disabled. 
   If encoders are enabled, the robot uses odometry ticks instead. */
constexpr uint16_t AUTO_SPIN_DIAGONAL_MS = 500;   // 45 degrees
constexpr uint16_t AUTO_SPIN_SIDE_MS     = 1000;  // 90 degrees

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

#if PID_CLOSED_LOOP_DEFAULT && !ENABLE_ENCODERS
    #error "Closed-loop PID requires encoders (ENABLE_ENCODERS)"
#endif