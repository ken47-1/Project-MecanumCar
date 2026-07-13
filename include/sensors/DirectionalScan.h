/* ==================== DirectionalScan.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/MotionCommand.h"
#include "sensors/Sensors.h"

/* =============== TYPES =============== */
struct SweepResult {
    uint16_t front_left;
    uint16_t front;
    uint16_t front_right;
    uint16_t left;
    uint16_t right;
    uint8_t  clear_mask;
};

constexpr uint8_t SWEEP_CLEAR_FRONT       = (1 << 0);
constexpr uint8_t SWEEP_CLEAR_FRONT_LEFT  = (1 << 1);
constexpr uint8_t SWEEP_CLEAR_FRONT_RIGHT = (1 << 2);
constexpr uint8_t SWEEP_CLEAR_LEFT        = (1 << 3);
constexpr uint8_t SWEEP_CLEAR_RIGHT       = (1 << 4);

/* =============== API =============== */
namespace DirectionalScan {
    void init();
    void reset();

    // Tracking mode — follows movement direction
    void update(const MotionCommand& cmd);
    ScanDir current_scan_dir();

    // Sweep mode — scans all angles, returns results
    // Call start_sweep() once, then poll sweep_ready() each loop.
    // When ready, call get_sweep_result() to retrieve distances.
    void start_sweep();
    bool sweep_ready();
    SweepResult get_sweep_result();
    void update_sweep();   // call each loop while sweeping
}
