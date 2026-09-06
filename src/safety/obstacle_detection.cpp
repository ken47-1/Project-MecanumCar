/* ==================== obstacle_detection.cpp ==================== */
#include "config/Config.h"
#include "safety/obstacle_detection.h"

#if ENABLE_OBSTACLE_AVOIDANCE

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */
#include "sensors/ultrasonic.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace ObstacleDetection {

/* =============== INTERNAL STATE =============== */
/* ============ FRONT SENSOR ============ */
static bool     front_in_slow = false;
static bool     front_in_stop = false;
static uint32_t front_last_clear_slow_ms = 0;
static uint32_t front_last_clear_stop_ms = 0;

/* ============ REAR SENSOR ============ */
static bool     rear_in_slow = false;
static bool     rear_in_stop = false;
static uint32_t rear_last_clear_slow_ms = 0;
static uint32_t rear_last_clear_stop_ms = 0;

/* =============== INTERNAL HELPERS =============== */
/* ============ HYSTERESIS ============ */
static void update_zone(
    uint16_t distance,
    uint16_t enter_threshold,
    uint16_t exit_threshold,
    bool& in_zone,
    uint32_t& last_clear_ms
) {
    if (distance == 0) {
        return;  // invalid reading, hold state
    }

    bool triggered = (distance <= enter_threshold);
    bool should_exit = (distance > exit_threshold);

    if (triggered) {
        in_zone = true;
        last_clear_ms = 0;  // reset clear timer
    } else if (should_exit && in_zone) {
        // Start/update clear timer
        if (last_clear_ms == 0) {
            last_clear_ms = millis();
        }
        
        // Hold for OA_CLEAR_HOLD_MS before clearing
        if (millis() - last_clear_ms >= OA_CLEAR_HOLD_MS) {
            in_zone = false;
        }
    }
}

/* =============== PUBLIC API =============== */
void init() {
    front_in_slow = false;
    front_in_stop = false;
    front_last_clear_slow_ms = 0;
    front_last_clear_stop_ms = 0;

    rear_in_slow = false;
    rear_in_stop = false;
    rear_last_clear_slow_ms = 0;
    rear_last_clear_stop_ms = 0;

    Comms::system.println("ObstacleDetection INIT");
}

void update() {
    #if ENABLE_ULTRASONIC_FRONT
    uint16_t front_dist = Ultrasonic::get_front_distance_raw_cm();
    #else
    // Return clear
    uint16_t front_dist = 999;
    #endif
    #if ENABLE_ULTRASONIC_REAR
    uint16_t rear_dist = Ultrasonic::get_rear_distance_raw_cm();
    #else
    // Return clear
    uint16_t rear_dist = 999;
    #endif

    #if DEBUG_SENSORS
        char buf[50];
        snprintf(buf, sizeof(buf), "Front: %u cm | Rear: %u cm", front_dist, rear_dist);
        Comms::system.println(buf);
    #endif

    /* ===== FRONT ZONES ===== */
    update_zone(front_dist, FRONT_SLOW_ENTER_CM, FRONT_SLOW_EXIT_CM,
                front_in_slow, front_last_clear_slow_ms);
    update_zone(front_dist, FRONT_STOP_ENTER_CM, FRONT_STOP_EXIT_CM,
                front_in_stop, front_last_clear_stop_ms);

    /* ===== REAR ZONES ===== */
    update_zone(rear_dist, REAR_SLOW_ENTER_CM, REAR_SLOW_EXIT_CM,
                rear_in_slow, rear_last_clear_slow_ms);
    update_zone(rear_dist, REAR_STOP_ENTER_CM, REAR_STOP_EXIT_CM,
                rear_in_stop, rear_last_clear_stop_ms);

    #if DEBUG_OA_REASON
        char buf[40];
        
        snprintf(buf, sizeof(buf), "Front slow: %d | stop: %d", front_in_slow, front_in_stop);
        Comms::system.println(buf);
        
        snprintf(buf, sizeof(buf), "Rear slow: %d | stop: %d", rear_in_slow, rear_in_stop);
        Comms::system.println(buf);
    #endif
}

Proximity get_front() {
    #if ENABLE_ULTRASONIC_FRONT
    return {
        Ultrasonic::get_front_distance_cm(),
        front_in_slow,
        front_in_stop
    };
    #else
    // Return clear
    return { 999, false, false };
    #endif
}

Proximity get_rear() {
    #if ENABLE_ULTRASONIC_REAR
    return {
        Ultrasonic::get_rear_distance_cm(),
        rear_in_slow,
        rear_in_stop
    };
    #else
    // Return clear
    return { 999, false, false };
    #endif
}

} // namespace ObstacleDetection

#endif // ENABLE_OBSTACLE_AVOIDANCE