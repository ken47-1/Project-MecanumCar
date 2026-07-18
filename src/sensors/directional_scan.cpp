/* ==================== directional_scan.cpp ==================== */
#include "sensors/directional_scan.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/comms.h"
#include "sensors/sensors.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace DirectionalScan {

/* =============== INTERNAL STATE =============== */
static ScanDir active_dir = ScanDir::FRONT;

/* ========= SWEEP STATE ========= */
enum class SweepPhase : uint8_t {
    IDLE,
    SETTLING,   // waiting for servo to reach position
    READING,    // taking distance reading
    DONE
};

static SweepPhase    sweep_phase    = SweepPhase::IDLE;
static uint8_t       sweep_step     = 0;
static unsigned long sweep_timer_ms = 0;
static SweepResult   sweep_result   = {};

// Sweep order: FRONT -> FRONT_LEFT -> FRONT_RIGHT -> LEFT -> RIGHT
static const ScanDir SWEEP_DIRS[] = {
    ScanDir::FRONT,
    ScanDir::FRONT_LEFT,
    ScanDir::FRONT_RIGHT,
    ScanDir::LEFT,
    ScanDir::RIGHT
};
static constexpr uint8_t SWEEP_COUNT = 5;

/* =============== INTERNAL HELPERS =============== */
/* ============ CLASSIFICATION ============ */
static ScanDir classify(const MotionCommand& cmd) {
    if (cmd.forward == 0 && cmd.strafe == 0) return ScanDir::FRONT;

    if (cmd.forward > 0 && cmd.strafe > 0) return ScanDir::FRONT_RIGHT;
    if (cmd.forward > 0 && cmd.strafe < 0) return ScanDir::FRONT_LEFT;

    if (cmd.forward > 0) return ScanDir::FRONT;
    if (cmd.strafe > 0)  return ScanDir::RIGHT;
    if (cmd.strafe < 0)  return ScanDir::LEFT;

    return ScanDir::FRONT;
}

/* ============ DATA MAPPING ============ */
static uint16_t* result_slot(uint8_t step) {
    switch (SWEEP_DIRS[step]) {
        case ScanDir::FRONT:       return &sweep_result.front;
        case ScanDir::FRONT_LEFT:  return &sweep_result.front_left;
        case ScanDir::FRONT_RIGHT: return &sweep_result.front_right;
        case ScanDir::LEFT:        return &sweep_result.left;
        case ScanDir::RIGHT:       return &sweep_result.right;
        default:                   return nullptr;
    }
}

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void reset() {
    active_dir   = ScanDir::FRONT;
    sweep_phase  = SweepPhase::IDLE;
    sweep_step   = 0;
    sweep_result = {};
    Sensors::scan_set_direction(active_dir);
}

void init() {
    reset();
    Comms::system.println("DirectionalScan INIT");
    Comms::system.println("- Modes: Tracking, Sweep");
}

/* ============ MODE: TRACKING ============ */
void update(const MotionCommand& cmd) {
    /* Tracking mode only active when not performing a sweep */
    if (sweep_phase != SweepPhase::IDLE) return;

    ScanDir next = classify(cmd);
    if (next != active_dir) {
        active_dir = next;
        Sensors::scan_set_direction(active_dir);
    }
}

ScanDir current_scan_dir() {
    return active_dir;
}

/* ============ MODE: SWEEP ============ */
void start_sweep() {
    sweep_step     = 0;
    sweep_result   = {};
    sweep_phase    = SweepPhase::SETTLING;
    sweep_timer_ms = millis();

    Sensors::scan_set_direction(SWEEP_DIRS[0]);
}

void update_sweep() {
    if (sweep_phase == SweepPhase::IDLE || sweep_phase == SweepPhase::DONE) return;

    unsigned long now = millis();

    /* --- Step 1: Wait for Servo --- */
    if (sweep_phase == SweepPhase::SETTLING) {
        if (now - sweep_timer_ms >= SCAN_SERVO_SETTLE_MS) {
            sweep_phase = SweepPhase::READING;
        }
        return;
    }

    /* --- Step 2: Sample Distance --- */
    if (sweep_phase == SweepPhase::READING) {
        /* Raw ping — servo just settled, EMA would blend in old-angle readings */
        uint16_t dist = Sensors::get_front_distance_raw_cm();
        if (dist == 0) dist = 999;

        uint16_t* slot = result_slot(sweep_step);
        if (slot) *slot = dist;

        /* Update clear mask if path is beyond stop threshold */
        if (dist > FRONT_STOP_ENTER_CM) {
            switch (SWEEP_DIRS[sweep_step]) {
                case ScanDir::FRONT:       sweep_result.clear_mask |= SWEEP_CLEAR_FRONT;       break;
                case ScanDir::FRONT_LEFT:  sweep_result.clear_mask |= SWEEP_CLEAR_FRONT_LEFT;  break;
                case ScanDir::FRONT_RIGHT: sweep_result.clear_mask |= SWEEP_CLEAR_FRONT_RIGHT; break;
                case ScanDir::LEFT:        sweep_result.clear_mask |= SWEEP_CLEAR_LEFT;        break;
                case ScanDir::RIGHT:       sweep_result.clear_mask |= SWEEP_CLEAR_RIGHT;       break;
                default: break;
            }
        }

        sweep_step++;

        /* Check for completion */
        if (sweep_step >= SWEEP_COUNT) {
            sweep_phase = SweepPhase::DONE;
            active_dir  = ScanDir::FRONT;
            Sensors::scan_set_direction(active_dir);
            return;
        }

        /* Move to next position */
        Sensors::scan_set_direction(SWEEP_DIRS[sweep_step]);
        sweep_phase    = SweepPhase::SETTLING;
        sweep_timer_ms = millis();
    }
}

bool sweep_ready() {
    return sweep_phase == SweepPhase::DONE;
}

SweepResult get_sweep_result() {
    sweep_phase = SweepPhase::IDLE;
    return sweep_result;
}

void set_angle(int angle) {
    /* Manual override for alignment during autonomous spin */
    if (angle == 0) {
        active_dir = ScanDir::FRONT;
        Sensors::scan_set_direction(ScanDir::FRONT);
    }
}

} // namespace DirectionalScan