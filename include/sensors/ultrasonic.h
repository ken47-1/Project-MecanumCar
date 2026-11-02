/* ==================== ultrasonic.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ CORE ============ */
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
    #if ENABLE_ULTRASONIC_FRONT
        uint16_t get_front_distance_cm();
    #else
        inline uint16_t get_front_distance_cm() { return DISTANCE_FALLBACK_CM; }
    #endif

    #if ENABLE_ULTRASONIC_REAR
        uint16_t get_rear_distance_cm();
    #else
        inline uint16_t get_rear_distance_cm() { return DISTANCE_FALLBACK_CM; }
    #endif

    /* Raw single ping */
    #if ENABLE_ULTRASONIC_FRONT
        uint16_t get_front_distance_raw_cm();
    #else
        inline uint16_t get_front_distance_raw_cm() { return DISTANCE_FALLBACK_CM; }
    #endif    

    #if ENABLE_ULTRASONIC_REAR
        uint16_t get_rear_distance_raw_cm();
    #else
        inline uint16_t get_rear_distance_raw_cm() { return DISTANCE_FALLBACK_CM; }
    #endif

    #if ENABLE_ULTRASONIC_FRONT
        uint16_t apply_front_ema(uint16_t raw);
    #else
        inline uint16_t apply_front_ema(uint16_t) { return DISTANCE_FALLBACK_CM; }
    #endif

    #if ENABLE_ULTRASONIC_REAR
        uint16_t apply_rear_ema(uint16_t raw);
    #else
        inline uint16_t apply_rear_ema(uint16_t) { return DISTANCE_FALLBACK_CM; }
    #endif

    void scan_set_direction(ScanDir dir);
    ScanDir scan_get_direction();
}
