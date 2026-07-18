/* ==================== motor_ramp.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <stdint.h>

/* =============== TYPES =============== */
struct MotorSet {
    float fl, fr, rl, rr;
};

/* =============== API =============== */
namespace MotorRamp {
    void reset();
    void set_target(const MotorSet& target);
    void update();
    MotorSet current();
}