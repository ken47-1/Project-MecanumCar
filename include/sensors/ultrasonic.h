/* ==================== ultrasonic.h ==================== */
#pragma once

#include <stdint.h>

/* =============== TYPES =============== */
enum class ScanDir : uint8_t {
    NONE,
    FRONT,
    FRONT_LEFT,
    FRONT_RIGHT,
    LEFT,
    RIGHT
};

/* =============== API =============== */
namespace Ultrasonic {
    void init();

    /* EMA-filtered readings */
    uint16_t get_front_distance_cm();

    #if ENABLE_ULTRASONIC_REAR
        uint16_t get_rear_distance_cm();
    #endif

    /* Raw single ping */
    uint16_t get_front_distance_raw_cm();

    #if ENABLE_ULTRASONIC_REAR
        uint16_t get_rear_distance_raw_cm();
    #endif

    void scan_set_direction(ScanDir dir);
}