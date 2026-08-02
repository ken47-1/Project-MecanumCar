/* ==================== ultrasonic.cpp ==================== */
#include "config/Config.h"
#include "config/HardwareConfig.h"
#include "sensors/ultrasonic.h"

#if ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */
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

    Comms::system.println("Sensors INIT");

    char buf[40];

    #if ENABLE_ULTRASONIC_FRONT
        snprintf(buf, sizeof(buf), "- Front: EMA (a = %.2f)", ULTRASONIC_EMA_ALPHA_FRONT);
        Comms::system.println(buf);
    #endif
    
    #if ENABLE_ULTRASONIC_REAR
        snprintf(buf, sizeof(buf), "- Rear:  EMA (a = %.2f)", ULTRASONIC_EMA_ALPHA_REAR);
        Comms::system.println(buf);
    #endif
}

/* ============ TELEMETRY ============ */
/* ------ EMA Filtered (Continuous) ------ */
uint16_t get_front_distance_cm() {
    uint16_t raw = front_sonar.ping_cm();

    if (raw == 0) {
        if (!front_ema_initialized) return 999;        // No history → clear
        return (uint16_t)(front_filtered_cm + 0.5f);   // Hold last value
    }

    if (!front_ema_initialized) {
        front_filtered_cm = raw;
        front_ema_initialized = true;
    } else {
        front_filtered_cm += ULTRASONIC_EMA_ALPHA_FRONT * ((float)raw - front_filtered_cm);
    }
    return (uint16_t)(front_filtered_cm + 0.5f);
}

#if ENABLE_ULTRASONIC_REAR
uint16_t get_rear_distance_cm() {
    uint16_t raw = rear_sonar.ping_cm();
    
    if (raw == 0) {
        if (!rear_ema_initialized) return 999;        // No history → clear
        return (uint16_t)(rear_filtered_cm + 0.5f);   // Hold last value
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
uint16_t get_front_distance_raw_cm() {
    uint16_t raw = front_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}

#if ENABLE_ULTRASONIC_REAR
uint16_t get_rear_distance_raw_cm() {
    uint16_t raw = rear_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}
#endif

/* ============ ACTUATION ============ */
void scan_set_direction(ScanDir dir) {
    static ScanDir last_dir = ScanDir::NONE;

    #if !ENABLE_SERVO
        return;
    #endif

    if (!servo_ready) {
        return;
    }
    
    if (dir == last_dir || dir == ScanDir::NONE) {
        return;
    }

    last_dir = dir;

    /* --- Servo Write --- */
    switch (dir) {
        case ScanDir::FRONT:       scan_servo.write(SERVO_CENTER);      break;
        case ScanDir::FRONT_LEFT:  scan_servo.write(SERVO_FRONT_LEFT);  break;
        case ScanDir::FRONT_RIGHT: scan_servo.write(SERVO_FRONT_RIGHT); break;
        case ScanDir::LEFT:        scan_servo.write(SERVO_LEFT);        break;
        case ScanDir::RIGHT:       scan_servo.write(SERVO_RIGHT);       break;
        default: break;
    }
}

} // namespace Ultrasonic

#endif // ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR