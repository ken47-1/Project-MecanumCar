/* ==================== motor_ramp.cpp ==================== */
#include "control/motor_ramp.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
static MotorSet cur = {0.0f, 0.0f, 0.0f, 0.0f};
static MotorSet tgt = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t last_update_ms = millis();

/* =============== INTERNAL HELPERS =============== */
static float ramp_toward(
    float current,
    float target,
    float max_up_delta,
    float max_down_delta
) {
    float delta = target - current;

    // Decelerating = moving toward zero (magnitudes getting smaller)
    bool decelerating = (fabsf(target) < fabsf(current));
    float limit = decelerating ? max_down_delta : max_up_delta;

    if (delta > limit)        delta =  limit;
    else if (delta < -limit)  delta = -limit;

    return current + delta;
}

/* =============== PUBLIC API =============== */
namespace MotorRamp {

void reset() {
    cur = {0.0f, 0.0f, 0.0f, 0.0f};
    tgt = {0.0f, 0.0f, 0.0f, 0.0f};
    last_update_ms = millis();
}

void set_target(const MotorSet& target) {
    // clamp intent at subsystem boundary
    tgt.fl = constrain(target.fl, -1.0f, 1.0f);
    tgt.fr = constrain(target.fr, -1.0f, 1.0f);
    tgt.rl = constrain(target.rl, -1.0f, 1.0f);
    tgt.rr = constrain(target.rr, -1.0f, 1.0f);
}

void update() {
    if (RAMP_UP_TIME_MS <= 0 || RAMP_DOWN_TIME_MS <= 0) {
        cur = tgt;
        last_update_ms = millis();
        return;
    }

    uint32_t now = millis();
    uint32_t dt_ms = now - last_update_ms;
    if (dt_ms == 0) return;

    // prevent stall spikes
    if (dt_ms > 100) dt_ms = 100;

    last_update_ms = now;

    float dt_sec = dt_ms * 0.001f;

    // max allowed intent change this update
    float max_up_delta =
        dt_sec / (RAMP_UP_TIME_MS * 0.001f);

    float max_down_delta =
        dt_sec / (RAMP_DOWN_TIME_MS * 0.001f);

    cur.fl = constrain(ramp_toward(cur.fl, tgt.fl, max_up_delta, max_down_delta), -1.0f, 1.0f);
    cur.fr = constrain(ramp_toward(cur.fr, tgt.fr, max_up_delta, max_down_delta), -1.0f, 1.0f);
    cur.rl = constrain(ramp_toward(cur.rl, tgt.rl, max_up_delta, max_down_delta), -1.0f, 1.0f);
    cur.rr = constrain(ramp_toward(cur.rr, tgt.rr, max_up_delta, max_down_delta), -1.0f, 1.0f);
}

MotorSet current() {
    return cur;
}

} // namespace MotorRamp