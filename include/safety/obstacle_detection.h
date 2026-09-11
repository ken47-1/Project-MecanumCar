/* ==================== obstacle_detection.h ==================== */
#pragma once

#include "config/Config.h"

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <stdint.h>

/* =============== TYPES =============== */
/* ============ STRUCTS ============ */
struct Proximity {
    uint16_t distance_cm;   // EMA-filtered reading (telemetry)
    bool in_slow_zone;      // Derived from RAW reading (stable via hysteresis)
    bool in_stop_zone;      // Derived from RAW reading (stable via hysteresis)
};

#if ENABLE_OBSTACLE_AVOIDANCE

/* =============== API =============== */
namespace ObstacleDetection {
    void init();
    void update();
    Proximity get_front();
    Proximity get_rear();
}

#else

/* =============== API =============== */
namespace ObstacleDetection {
    inline void init() {}
    inline void update() {}
    inline Proximity get_front() { return { 999, false, false }; }
    inline Proximity get_rear()  { return { 999, false, false }; }
}

#endif