/* ==================== motor_pid.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= CONTROL ========= */
#include "control/motor_ramp.h"

#if ENABLE_ENCODERS

/* =============== TYPES =============== */
/* ============ STRUCTS ============ */
struct WheelPID {
    float target;    /* intent [-1, 1] */
    float measured;  /* RPM / wheel_max_rpm [-1, 1] */
    float error;     /* target - measured */
    float output;    /* PWM fraction sent to the motor [-1, 1] */
};

/* =============== API =============== */
namespace MotorPID {
    /* --------- Lifecycle --------- */
    void reset();

    /* --------- Control --------- */
    MotorSet apply(const MotorSet& intent);

    /* --------- Mode --------- */
    void set_closed_loop(bool enabled);
    bool is_closed_loop();

    /* --------- Telemetry --------- */
    WheelPID get_wheel(uint8_t idx);
}

#else

/* =============== STUBS =============== */
namespace MotorPID {
    struct WheelPID { float target, measured, error, output; };
    inline void reset() {}
    inline MotorSet apply(const MotorSet& intent) { return intent; }
    inline void set_closed_loop(bool) {}
    inline bool is_closed_loop() { return false; }
    inline WheelPID get_wheel(uint8_t) { return {}; }
}

#endif // ENABLE_ENCODERS