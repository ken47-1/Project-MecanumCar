/* ==================== ultrasonic.cpp ==================== */
#include "sensors/ultrasonic.h"

#if ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"
#include "config/HardwareConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ============ THIRD-PARTY ============ */
#include <UltraPing.h>

/* ============ CORE ============ */
#include <Arduino.h>
#include <Servo.h>

namespace Ultrasonic {

/* =============== INTERNAL STATE =============== */
/* ============ HARDWARE ============ */
static Servo scan_servo;
static bool  servo_ready = false;

static UltraPing front_sonar(
    SR04_FRONT_TRIG_PIN,
    SR04_FRONT_ECHO_PIN,
    90
);

#if ENABLE_ULTRASONIC_REAR
static UltraPing rear_sonar(
    SR04_REAR_TRIG_PIN,
    SR04_REAR_ECHO_PIN,
    90
);
#endif

/* ============ SERVO STATE ============ */
static ScanDir active_scan_dir = ScanDir::NONE;

/* ============ FILTERING ============ */
static float front_filtered_cm     = 0.0f;
static bool  front_ema_initialized  = false;

#if ENABLE_ULTRASONIC_REAR
static float rear_filtered_cm      = 0.0f;
static bool  rear_ema_initialized   = false;
#endif

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    #if ENABLE_SERVO
        scan_servo.attach(SCAN_SERVO_PIN);
        servo_ready = true;
        scan_set_direction(ScanDir::FRONT);
    #endif

    Comms::system.println(F("Sensors INIT"));

    char buf[40];

    #if ENABLE_ULTRASONIC_FRONT
        snprintf(buf, sizeof(buf), "- Front: EMA (a = %.2f)", (double)ULTRASONIC_EMA_ALPHA_FRONT);
        Comms::system.println(buf);
    #endif
    
    #if ENABLE_ULTRASONIC_REAR
        snprintf(buf, sizeof(buf), "- Rear:  EMA (a = %.2f)", (double)ULTRASONIC_EMA_ALPHA_REAR);
        Comms::system.println(buf);
    #endif
}

/* ============ TELEMETRY ============ */
/* ------ EMA Filtered (Continuous) ------ */
#if ENABLE_ULTRASONIC_FRONT
uint16_t get_front_distance_cm() {
    return apply_front_ema(front_sonar.ping_cm());
}

uint16_t apply_front_ema(uint16_t raw) {
    if (raw == 0 || raw > 400) {
        if (!front_ema_initialized) return 999;
        return (uint16_t)(front_filtered_cm + 0.5f);
    }
    if (!front_ema_initialized) {
        front_filtered_cm = raw;
        front_ema_initialized = true;
    } else {
        front_filtered_cm += ULTRASONIC_EMA_ALPHA_FRONT * ((float)raw - front_filtered_cm);
    }
    return (uint16_t)(front_filtered_cm + 0.5f);
}
#endif

#if ENABLE_ULTRASONIC_REAR
uint16_t get_rear_distance_cm() {
    return apply_rear_ema(rear_sonar.ping_cm());
}

uint16_t apply_rear_ema(uint16_t raw) {
    if (raw == 0 || raw > 400) {
        if (!rear_ema_initialized) return 999;
        return (uint16_t)(rear_filtered_cm + 0.5f);
    }
    if (!rear_ema_initialized) {
        rear_filtered_cm = raw;
        rear_ema_initialized = true;
    } else {
        rear_filtered_cm += ULTRASONIC_EMA_ALPHA_REAR * ((float)raw - rear_filtered_cm);
    }
    return (uint16_t)(rear_filtered_cm + 0.5f);
}
#endif

/* ------ Raw Access (State Transitions) ------ */
#if ENABLE_ULTRASONIC_FRONT
uint16_t get_front_distance_raw_cm() {
    uint16_t raw = front_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}
#endif

#if ENABLE_ULTRASONIC_REAR
uint16_t get_rear_distance_raw_cm() {
    uint16_t raw = rear_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}
#endif

/* ============ ACTUATION ============ */
void scan_set_direction(ScanDir dir) {
    if (dir == ScanDir::NONE) {
        return;
    }

    if (dir == active_scan_dir) {
        return;
    }

    active_scan_dir = dir;

    #if ENABLE_SERVO
        if (servo_ready) {
            switch (dir) {
                case ScanDir::FRONT:       scan_servo.write(SERVO_CENTER);      break;
                case ScanDir::FRONT_LEFT:  scan_servo.write(SERVO_FRONT_LEFT);  break;
                case ScanDir::FRONT_RIGHT: scan_servo.write(SERVO_FRONT_RIGHT); break;
                case ScanDir::LEFT:        scan_servo.write(SERVO_LEFT);        break;
                case ScanDir::RIGHT:       scan_servo.write(SERVO_RIGHT);       break;
                default: break;
            }
        }
    #endif
}

ScanDir scan_get_direction() {
    return active_scan_dir;
}

} // namespace Ultrasonic

#endif // ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR
