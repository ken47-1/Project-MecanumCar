/* ==================== motor_pid.cpp ==================== */
#include "control/motor_pid.h"

#if ENABLE_ENCODERS

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= SENSORS ========= */
#include "sensors/encoder.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
/* ============ PID STATE ============ */
struct PIDState {
    float integral;
    float last_meas;
};

static PIDState pid[4] = {};

/* ============ OUTPUT CACHE ============ */
static MotorSet      last_out    = {};
static unsigned long last_run_ms = 0;

/* ============ MODE ============ */
static bool closed_loop = PID_CLOSED_LOOP_DEFAULT;

/* =============== INTERNAL HELPERS =============== */
static float compute_pid(uint8_t idx, float error, float measured_frac) {
    constexpr float DT = PID_PERIOD_MS / 1000.0f;

    PIDState& s = pid[idx];

    /* Integral with anti-windup. */
    s.integral += error * DT;
    s.integral  = constrain(s.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    /* Derivative on measurement (no kick on setpoint change). */
    float d_meas = (measured_frac - s.last_meas) / DT;
    s.last_meas  = measured_frac;

    return PID_KP * error + PID_KI * s.integral - PID_KD * d_meas;
}

static float apply_one(uint8_t idx, float intent) {
    if (intent == 0.0f) {
        pid[idx].integral  = 0.0f;
        pid[idx].last_meas = 0.0f;
        return 0.0f;
    }

    /* Normalize to [-1, 1] so PID gains stay in a stable range. */
    float measured_frac = Encoder::get_rpm(idx) / MOTOR_MAX_RPM;
    float error         = intent - measured_frac;

    float correction = compute_pid(idx, error, measured_frac);
    float output     = constrain(intent + correction, -1.0f, 1.0f);

    /* Never let the PID reverse against the commanded direction. */
    if (intent > 0.0f && output < 0.0f) {
        output = 0.0f;
    }
    if (intent < 0.0f && output > 0.0f) {
        output = 0.0f;
    }

    return output;
}

/* =============== PUBLIC API =============== */
namespace MotorPID {

void reset() {
    for (uint8_t i = 0; i < 4; i++) {
        pid[i].integral  = 0.0f;
        pid[i].last_meas = 0.0f;
    }
    last_out    = {};
    last_run_ms = 0;
}

MotorSet apply(const MotorSet& intent) {
    /* Open loop: pass through, no rate limit, no PID. */
    if (!closed_loop) {
        return intent;
    }

    /* Closed loop rate limit. 50 Hz is enough for the wheel dynamics. */
    if (millis() - last_run_ms < PID_PERIOD_MS) {
        return last_out;
    }
    last_run_ms = millis();

    last_out.fl = apply_one(0, intent.fl);
    last_out.fr = apply_one(1, intent.fr);
    last_out.rl = apply_one(2, intent.rl);
    last_out.rr = apply_one(3, intent.rr);

    return last_out;
}

void set_closed_loop(bool enabled) {
    if (enabled == closed_loop) {
        return;
    }
    /* Reset integral and derivative state so the switch does not kick. */
    reset();
    closed_loop = enabled;
}

bool is_closed_loop() {
    return closed_loop;
}

}

#endif // ENABLE_ENCODERS