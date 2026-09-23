/* ==================== motor_pid.cpp ==================== */
#include "control/motor_pid.h"

#if ENABLE_ENCODERS

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= CONTROL ========= */
#include "control/motor_fault.h"

/* ========= SENSORS ========= */
#include "sensors/encoder.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
/* ============ PID STATE ============ */
struct PIDState {
    float    integral;
    float    last_meas;
    float    target;
    float    measured;
    float    error;
    float    output;
    int8_t   last_dir;
    uint16_t stall_ticks;
};

static PIDState pid[4] = {};

/* ============ OUTPUT CACHE ============ */
static MotorSet      last_out    = {};
static uint32_t      last_run_us = 0;

/* ============ MODE ============ */
static bool closed_loop = PID_CLOSED_LOOP_DEFAULT;

/* =============== INTERNAL HELPERS =============== */
/* ============ PER-WHEEL CEILING ============ */
static inline float wheel_max_rpm(uint8_t idx) {
    switch (idx) {
        case 0:  return MOTOR_MAX_RPM_FL;
        case 1:  return MOTOR_MAX_RPM_FR;
        case 2:  return MOTOR_MAX_RPM_RL;
        default: return MOTOR_MAX_RPM_RR;
    }
}

/* ============ ONE WHEEL ============ */
static float apply_one(uint8_t idx, float intent, float dt) {
    float max_rpm = wheel_max_rpm(idx);
    if (max_rpm < 1.0f) {
        /* Misconfigured. Fail safe: pass intent through. */
        return intent;
    }

    PIDState& s = pid[idx];

    /* --- Stopped --- */
    if (intent == 0.0f) {
        s = {};
        return 0.0f;
    }

    /* --- Direction Reversal --- */
    int8_t dir = (intent > 0.0f) ? 1 : -1;
    if (dir != s.last_dir) {
        s.integral    = 0.0f;
        s.last_meas   = 0.0f;
        s.stall_ticks = 0;
        s.last_dir    = dir;
    }

    /* --- Measure --- */
    float rpm           = Encoder::get_rpm(idx);
    float measured_frac = rpm / max_rpm;
    float error         = intent - measured_frac;

    s.target   = intent;
    s.measured = measured_frac;
    s.error    = error;

    /* --- Proportional + Derivative --- */
    float d_meas       = (measured_frac - s.last_meas) / dt;
    s.last_meas        = measured_frac;
    float proportional = PID_KP * error;
    float derivative   = -PID_KD * d_meas;

    /* --- Integral (conditional: skip if saturated against the error) --- */
    float tentative = intent + proportional + PID_KI * s.integral + derivative;
    bool  sat_high  = (tentative >=  1.0f && error > 0.0f);
    bool  sat_low   = (tentative <= -1.0f && error < 0.0f);
    if (!sat_high && !sat_low) {
        s.integral += error * dt;
        s.integral  = constrain(s.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    }

    /* --- Final Output --- */
    float output = constrain(
        intent + proportional + PID_KI * s.integral + derivative,
        -1.0f, 1.0f
    );

    /* Never reverse against intent. */
    if (intent > 0.0f && output < 0.0f) output = 0.0f;
    if (intent < 0.0f && output > 0.0f) output = 0.0f;

    /* Motor deadband floor, if configured. */
    if (PID_MIN_OUTPUT > 0.0f && fabsf(output) < PID_MIN_OUTPUT) {
        output = (intent > 0.0f) ? PID_MIN_OUTPUT : -PID_MIN_OUTPUT;
    }

    s.output = output;

    /* --- Encoder Stall Detection --- */
    if (fabsf(intent) >= PID_STALL_INTENT_MIN && rpm == 0.0f) {
        if (++s.stall_ticks >= PID_STALL_TICKS) {
            MotorFault::trigger(MotorFaultReason::SENSOR_FAIL);
            return 0.0f;
        }
    } else {
        s.stall_ticks = 0;
    }

    return output;
}

/* =============== PUBLIC API =============== */
namespace MotorPID {

void reset() {
    for (uint8_t i = 0; i < 4; i++) {
        pid[i] = {};
    }
    last_out    = {};
    last_run_us = 0;
}

MotorSet apply(const MotorSet& intent) {
    /* Open loop: pass through. */
    if (!closed_loop) {
        return intent;
    }

    /* --- Deterministic Timing --- */
    uint32_t now_us = micros();
    uint32_t dt_us  = now_us - last_run_us;
    if (dt_us < (uint32_t)PID_PERIOD_MS * 1000UL) {
        return last_out;
    }
    last_run_us = now_us;

    float dt = dt_us * 1e-6f;
    if (dt > PID_DT_MAX) dt = PID_DT_MAX;

    last_out.fl = apply_one(0, intent.fl, dt);
    last_out.fr = apply_one(1, intent.fr, dt);
    last_out.rl = apply_one(2, intent.rl, dt);
    last_out.rr = apply_one(3, intent.rr, dt);

    return last_out;
}

void set_closed_loop(bool enabled) {
    if (enabled == closed_loop) {
        return;
    }
    reset();
    closed_loop = enabled;
}

bool is_closed_loop() {
    return closed_loop;
}

WheelPID get_wheel(uint8_t idx) {
    if (idx >= 4) {
        return {};
    }
    return {
        pid[idx].target,
        pid[idx].measured,
        pid[idx].error,
        pid[idx].output
    };
}

} // namespace MotorPID

#endif // ENABLE_ENCODERS
