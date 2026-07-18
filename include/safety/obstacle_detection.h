/* ==================== obstacle_detection.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ==================== THIRD-PARTY ==================== */
#include <stdint.h>

/* =============== TYPES =============== */
struct Proximity {
    uint16_t distance_cm;  // 0 = invalid/no reading
    bool in_slow_zone;     // within slow threshold (scale down authority)
    bool in_stop_zone;     // within stop threshold (force backoff or block)
};

/* =============== API =============== */
namespace ObstacleDetection {
    void init();
    void update();  // call each loop — reads sensors, applies hysteresis
    
    Proximity get_front();
    Proximity get_rear();
}
