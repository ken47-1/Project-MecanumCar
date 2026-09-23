/* ==================== motor_ramp.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ CORE ============ */
#include <stdint.h>

/* =============== TYPES =============== */
/* ============ STRUCTS ============ */
struct MotorSet {
    float fl, fr, rl, rr;
};

/* =============== API =============== */
namespace MotorRamp {
    void reset();
    void set_target(const MotorSet& target);
    void snap_to_target();
    void update();
    MotorSet current();
    MotorSet target();
}
