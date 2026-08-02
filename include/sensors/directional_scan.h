/* ==================== directional_scan.h ==================== */
#pragma once

#include "config/Config.h"

#if ENABLE_DIRECTIONAL_SCAN

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */
#include "control/motion_command.h"
#include "sensors/ultrasonic.h"

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
    /* --------- Lifecycle --------- */
    void init();
    void reset();

    /* --------- Tracking --------- */
    void update(const MotionCommand& cmd);
    ScanDir current_scan_dir();

    /* --------- Sweep --------- */
    void start_sweep();
    bool sweep_ready();
    SweepResult get_sweep_result();
    void update_sweep();

    /* --------- Status --------- */
    bool is_settled();
}

#endif // ENABLE_DIRECTIONAL_SCAN