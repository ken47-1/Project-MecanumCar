/* ==================== motor_pid.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= CONTROL ========= */
#include "control/motor_ramp.h"

#if ENABLE_ENCODERS

/* =============== API =============== */
namespace MotorPID {
    /* --------- Lifecycle --------- */
    void reset();

    /* --------- Control --------- */
    MotorSet apply(const MotorSet& intent);

    /* --------- Mode --------- */
    /* Runtime toggle between closed loop (PID active) and open loop
       (raw intent passthrough). */
    void set_closed_loop(bool enabled);
    bool is_closed_loop();
}

#else

/* =============== STUBS =============== */
namespace MotorPID {
    inline void reset() {}
    inline MotorSet apply(const MotorSet& intent) { return intent; }
    inline void set_closed_loop(bool) {}
    inline bool is_closed_loop() { return false; }
}

#endif // ENABLE_ENCODERS
