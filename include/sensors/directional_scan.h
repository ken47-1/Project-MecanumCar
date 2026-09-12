/* ==================== directional_scan.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= CONTROL ========= */
#include "control/motion_command.h"

/* ========= SENSORS ========= */
#include "sensors/ultrasonic.h"

/* =============== TYPES =============== */
/* ============ STRUCTS ============ */
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

#if ENABLE_DIRECTIONAL_SCAN

/* =============== API =============== */
namespace DirectionalScan {
    void init();
    void reset();
    void update(const MotionCommand& cmd);
    ScanDir current_scan_dir();
    void start_sweep();
    bool sweep_ready();
    SweepResult get_sweep_result();
    void update_sweep();
    bool is_settled();
}

#else

/* =============== API =============== */
namespace DirectionalScan {
    inline void init() {}
    inline void reset() {}
    inline void update(const MotionCommand&) {}
    inline ScanDir current_scan_dir() { return ScanDir::NONE; }
    inline void start_sweep() {}
    inline bool sweep_ready() { return false; }
    inline SweepResult get_sweep_result() { return {}; }
    inline void update_sweep() {}
    inline bool is_settled() { return true; }
}

#endif // ENABLE_DIRECTIONAL_SCAN