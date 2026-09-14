/* ==================== motor_control.cpp ==================== */
#include "control/motor_control.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"
#include "log/log.h"

/* ========= CONTROL ========= */
#include "control/motor_fault.h"
#include "control/motor_ramp.h"
#include "control/motor_pid.h"

/* ========= SAFETY ========= */
#include "safety/motion_policy.h"
#include "safety/safety_manager.h"

/* ========= SENSORS ========= */
#include "sensors/encoder.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace MotorControl {

/* =============== INTERNAL STATE =============== */
/* ========= MOTOR HANDLES ========= */
static Adafruit_DCMotor* motor_fl = nullptr;
static Adafruit_DCMotor* motor_fr = nullptr;
static Adafruit_DCMotor* motor_rl = nullptr;
static Adafruit_DCMotor* motor_rr = nullptr;

/* ========= STOP STATE ========= */
static bool motors_stopped = true;

/* =============== INTERNAL HELPERS =============== */
static inline uint16_t float_to_pwm(float v) {
    v = constrain(v, -1.0f, 1.0f);

    // +0.5f = round-to-nearest instead of truncation
    return (uint16_t)(fabsf(v) * PWM_MAX + 0.5f);
}

static void drive_one_motor(Adafruit_DCMotor* m, float value) {
    if (!m) {
        return;
    }

    if (value > 0.0f)       m->run(FORWARD);
    else if (value < 0.0f)  m->run(BACKWARD);
    else {
        m->run(RELEASE);
        m->setSpeedFine(0);
        return;
    }

    m->setSpeedFine(float_to_pwm(value));
}

static inline int8_t sign_i8(float v) {
    if (v > 0.01f)  return 1;
    if (v < -0.01f) return -1;
    return 0;
}

/* =============== PUBLIC API =============== */
void init(MotorHardware& hw) {
    if (!hw.ready()) {
        MotorFault::trigger(MotorFaultReason::INTERNAL_ERROR);
        return;
    }

    motor_fl = hw.get(MotorId::FL);
    motor_fr = hw.get(MotorId::FR);
    motor_rl = hw.get(MotorId::RL);
    motor_rr = hw.get(MotorId::RR);

    hw.release_all();
    MotorRamp::reset();

    Comms::system.println(F("MotorControl INIT"));
}

void hard_stop() {
    MotorRamp::reset();
    MotorPID::reset();
    drive_one_motor(motor_fl, 0.0f);
    drive_one_motor(motor_fr, 0.0f);
    drive_one_motor(motor_rl, 0.0f);
    drive_one_motor(motor_rr, 0.0f);

    motors_stopped = true;
}

/* ------ INPUT ------ */
void apply_command(const MotionCommand& cmd) {
    if (cmd.forward == 0.0f && cmd.strafe == 0.0f && cmd.rotate == 0.0f) {
        hard_stop();
        return;
    }    

    motors_stopped = false;
    
    /* --- SAFETY POLICY --- */
    // All safety checks, obstacle avoidance, and authority scaling in one place
    MotionCommand safe_cmd = MotionPolicy::apply_safety(cmd);

    /* --- MECANUM MIX --- */
    float fl = safe_cmd.forward + safe_cmd.strafe + safe_cmd.rotate;
    float fr = safe_cmd.forward - safe_cmd.strafe - safe_cmd.rotate;
    float rl = safe_cmd.forward - safe_cmd.strafe + safe_cmd.rotate;
    float rr = safe_cmd.forward + safe_cmd.strafe - safe_cmd.rotate;

    float max_mag = max(max(fabsf(fl), fabsf(fr)), max(fabsf(rl), fabsf(rr)));
    if (max_mag > 1.0f) {
        fl /= max_mag;
        fr /= max_mag;
        rl /= max_mag;
        rr /= max_mag;
    }

    MotorRamp::set_target({fl, fr, rl, rr});
}

void apply_command_instant(const MotionCommand& cmd) {
    if (cmd.forward == 0.0f && cmd.strafe == 0.0f && cmd.rotate == 0.0f) {
        hard_stop();
        return;
    }

    motors_stopped = false;

    /* --- SAFETY POLICY --- */
    // All safety checks, obstacle avoidance, and authority scaling in one place
    MotionCommand safe_cmd = MotionPolicy::apply_safety(cmd);

    /* --- MECANUM MIX --- */
    float fl = safe_cmd.forward + safe_cmd.strafe + safe_cmd.rotate;
    float fr = safe_cmd.forward - safe_cmd.strafe - safe_cmd.rotate;
    float rl = safe_cmd.forward - safe_cmd.strafe + safe_cmd.rotate;
    float rr = safe_cmd.forward + safe_cmd.strafe - safe_cmd.rotate;

    float max_mag = max(max(fabsf(fl), fabsf(fr)), max(fabsf(rl), fabsf(rr)));
    if (max_mag > 1.0f) {
        fl /= max_mag; fr /= max_mag; rl /= max_mag; rr /= max_mag;
    }

    MotorRamp::set_target({fl, fr, rl, rr});
    MotorRamp::snap_to_target();   /* <- new: skip the ramp curve */
    #if ENABLE_ENCODERS
        MotorSet out = MotorPID::apply({fl, fr, rl, rr});
        drive_one_motor(motor_fl, out.fl);
        drive_one_motor(motor_fr, out.fr);
        drive_one_motor(motor_rl, out.rl);
        drive_one_motor(motor_rr, out.rr);
    #else
        drive_one_motor(motor_fl, fl);
        drive_one_motor(motor_fr, fr);
        drive_one_motor(motor_rl, rl);
        drive_one_motor(motor_rr, rr);
    #endif
}

/* ------ UPDATE LOOP ------ */
void update() {
    if (motors_stopped) {
        return;
    }

    MotorSet cur = MotorRamp::current();
    MotorSet tgt = MotorRamp::target();

    if (tgt.fl == 0.0f && fabs(cur.fl) < 0.05f &&
        tgt.fr == 0.0f && fabs(cur.fr) < 0.05f &&
        tgt.rl == 0.0f && fabs(cur.rl) < 0.05f &&
        tgt.rr == 0.0f && fabs(cur.rr) < 0.05f) {
        hard_stop();
        return;
    }

    // HARD STOP on emergency or input loss
    SafetyState safety = SafetyManager::get_state();
    if (safety == SAFETY_EMERGENCY_STOP ||
        safety == SAFETY_INPUT_LOSS ||
        safety == SAFETY_CONNECTION_LOSS) {
        hard_stop();
        return;
    }

    #if ENABLE_ENCODERS
        Encoder::set_direction(0, sign_i8(cur.fl));
        Encoder::set_direction(1, sign_i8(cur.fr));
        Encoder::set_direction(2, sign_i8(cur.rl));
        Encoder::set_direction(3, sign_i8(cur.rr));

        MotorSet out = MotorPID::apply(cur);

        drive_one_motor(motor_fl, out.fl);
        drive_one_motor(motor_fr, out.fr);
        drive_one_motor(motor_rl, out.rl);
        drive_one_motor(motor_rr, out.rr);
    #else
        drive_one_motor(motor_fl, cur.fl);
        drive_one_motor(motor_fr, cur.fr);
        drive_one_motor(motor_rl, cur.rl);
        drive_one_motor(motor_rr, cur.rr);
    #endif

    #if ENABLE_ENCODERS
        static unsigned long last_speed_print = 0;
        if (millis() - last_speed_print >= 500) {
            last_speed_print = millis();
            LOG_D(Log::Ch::CH_PID, "FL=%d FR=%d RL=%d RR=%d",
                      (int)Encoder::get_rpm(0),
                      (int)Encoder::get_rpm(1),
                      (int)Encoder::get_rpm(2),
                      (int)Encoder::get_rpm(3));
        }
    #endif
}

/* ------ STATUS ------ */
bool is_moving() {
    return !motors_stopped;
}

} // namespace MotorControl