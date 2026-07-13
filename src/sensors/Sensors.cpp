/* ==================== Sensors.cpp ==================== */
#include "sensors/Sensors.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/Comms.h"

/* ============ THIRD-PARTY ============ */
#include <NewPing.h>

/* ============ CORE ============ */
#include <Arduino.h>
#include <Servo.h>

namespace Sensors {

/* =============== INTERNAL STATE =============== */
/* ============ HARDWARE ============ */
static Servo scan_servo;
static bool  servo_ready = false;

static NewPing front_sonar(
    SR04_FRONT_TRIG_PIN,
    SR04_FRONT_ECHO_PIN,
    200   // max distance (cm)
);

static NewPing rear_sonar(
    SR04_REAR_TRIG_PIN,
    SR04_REAR_ECHO_PIN,
    200   // max distance (cm)
);

/* ============ FILTERING ============ */
static float front_filtered_cm     = 0.0f;
static bool  front_ema_initialized  = false;

static float rear_filtered_cm      = 0.0f;
static bool  rear_ema_initialized   = false;

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    scan_servo.attach(SCAN_SERVO_PIN);
    servo_ready = true;

    scan_set_direction(ScanDir::FRONT);

    /* --- Startup Telemetry --- */
    Comms::system.println("Sensors INIT");
    
    String f_msg = String("- Front: EMA (a = ") + ULTRASONIC_EMA_ALPHA_FRONT + ")";
    String r_msg = String("- Rear:  EMA (a = ") + ULTRASONIC_EMA_ALPHA_REAR + ")";
    
    Comms::system.println(f_msg);
    Comms::system.println(r_msg);
}

/* ============ TELEMETRY ============ */
/* ------ EMA Filtered (Continuous) ------ */
uint16_t get_front_distance_cm() {
    uint16_t raw = front_sonar.ping_cm();
    if (raw == 0) return (uint16_t)(front_filtered_cm + 0.5f);

    if (!front_ema_initialized) {
        front_filtered_cm = raw;
        front_ema_initialized = true;
    } else {
        front_filtered_cm += ULTRASONIC_EMA_ALPHA_FRONT * ((float)raw - front_filtered_cm);
    }
    return (uint16_t)(front_filtered_cm + 0.5f);
}

uint16_t get_rear_distance_cm() {
    uint16_t raw = rear_sonar.ping_cm();
    if (raw == 0) return (uint16_t)(rear_filtered_cm + 0.5f);

    if (!rear_ema_initialized) {
        rear_filtered_cm = raw;
        rear_ema_initialized = true;
    } else {
        rear_filtered_cm += ULTRASONIC_EMA_ALPHA_REAR * ((float)raw - rear_filtered_cm);
    }
    return (uint16_t)(rear_filtered_cm + 0.5f);
}

/* ------ Raw Access (State Transitions) ------ */
uint16_t get_front_distance_raw_cm() {
    uint16_t raw = front_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}

uint16_t get_rear_distance_raw_cm() {
    uint16_t raw = rear_sonar.ping_cm();
    return (raw == 0) ? 999 : raw;
}

/* ============ ACTUATION ============ */
void scan_set_direction(ScanDir dir) {
    static ScanDir last_dir = ScanDir::NONE;

    if (!servo_ready) return;
    if (dir == last_dir || dir == ScanDir::NONE) return;

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

} // namespace Sensors