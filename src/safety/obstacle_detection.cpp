/* ==================== obstacle_detection.cpp ==================== */
#include "safety/obstacle_detection.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "sensors/sensors.h"
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
    if (distance == 0) return;  // invalid reading, hold state

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
    uint16_t front_dist = Sensors::get_front_distance_cm();
    uint16_t rear_dist = Sensors::get_rear_distance_cm();

    #if DEBUG_SENSORS
        Comms::system.print("Front: ");
        Comms::system.print(front_dist);
        Comms::system.print(" | Rear: ");
        Comms::system.println(rear_dist);
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
        Comms::system.print("Front slow: ");
        Comms::system.print(front_in_slow);
        Comms::system.print(" | stop: ");
        Comms::system.print(front_in_stop);
        Comms::system.print(" | Rear slow: ");
        Comms::system.print(rear_in_slow);
        Comms::system.print(" | stop: ");
        Comms::system.println(rear_in_stop);
    #endif
}

Proximity get_front() {
    return {
        Sensors::get_front_distance_cm(),
        front_in_slow,
        front_in_stop
    };
}

Proximity get_rear() {
    return {
        Sensors::get_rear_distance_cm(),
        rear_in_slow,
        rear_in_stop
    };
}

} // namespace ObstacleDetection
