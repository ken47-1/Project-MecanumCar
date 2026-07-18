/* ==================== sensors.h ==================== */
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
namespace Sensors {
    void init();

    /* EMA-filtered readings — use in continuous loop (ObstacleDetection) */
    uint16_t get_front_distance_cm();
    uint16_t get_rear_distance_cm();

    /* Raw single ping — use after servo settle during sweeps, bypasses EMA */
    uint16_t get_front_distance_raw_cm();
    uint16_t get_rear_distance_raw_cm();

    void scan_set_direction(ScanDir dir);
}
