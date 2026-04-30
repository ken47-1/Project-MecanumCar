/* ==================== AutonomousController.cpp ==================== */
#include "control/AutonomousController.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/Comms.h"
#include "control/MotionCommand.h"
#include "control/MotorControl.h"
#include "control/ModeManager.h"
#include "safety/ObstacleDetection.h"
#include "safety/SafetyManager.h"
#include "sensors/DirectionalScan.h"
#include "sensors/Sensors.h"
#include "input/InputWatchdog.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace AutonomousController {

/* =============== INTERNAL TYPES =============== */
/* ============ ENUMS ============ */
enum class AutoState : uint8_t {
    MOVING,         // Driving forward
    SCANNING,       // Performing servo sweep
    SPINNING,       // Rotating for a fixed duration
    BACKING_UP,     // Escape: reversing
    STUCK           // Boxed in everywhere
};

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static AutoState     state          = AutoState::MOVING;
static float         spin_direction = 0.0f; // -1.0 Left, 1.0 Right
static uint16_t      spin_limit_ms  = 0;
static unsigned long timer_ms       = 0;

/* =============== INTERNAL HELPERS =============== */
/* ============ NAVIGATION ============ */
static void enter(AutoState next) {
    state = next;
    timer_ms = millis();
}

/* --- Search all swept directions for the absolute widest path --- */
static uint16_t pick_best_spin_time(const SweepResult& r, float& dir_out) {
    
    struct PathOption {
        uint16_t dist;
        float    direction; // -1.0 Left, 1.0 Right
        uint16_t duration;  // Time to turn to this angle
    };

    /* Define turn candidates with their specific timing needs */
    PathOption candidates[] = {
        { r.front_left,  -1.0f, AUTO_SPIN_DIAGONAL_MS }, // 45 deg
        { r.front_right,  1.0f, AUTO_SPIN_DIAGONAL_MS }, // 45 deg
        { r.left,        -1.0f, AUTO_SPIN_SIDE_MS     }, // 90 deg
        { r.right,        1.0f, AUTO_SPIN_SIDE_MS     }  // 90 deg
    };

    uint16_t max_dist  = 0;
    uint16_t best_time = 0;
    float    best_dir  = 0.0f;

    /* --- Absolute Best Search --- */
    for (uint8_t i = 0; i < 4; i++) {
        if (candidates[i].dist > max_dist) {
            max_dist  = candidates[i].dist;
            best_dir  = candidates[i].direction;
            best_time = candidates[i].duration;
        }
    }

    /* --- Path Clearance Check --- */
    // If the best available path is still narrow, return 0 to trigger escape
    if (max_dist < FRONT_SLOW_EXIT_CM) {
        dir_out = 0.0f;
        return 0;
    }

    dir_out = best_dir;
    return best_time;
}

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void reset() {
    state = AutoState::MOVING;
    DirectionalScan::reset();
}

/* ============ LOGIC LOOP ============ */
void update(InputWatchdog& watchdog) {
    /* --- Mode Guard --- */
    if (!ModeManager::is_autonomous()) return;

    /* --- Automatic Authority --- */
    watchdog.feed();

    /* --- Global Safety Check --- */
    if (SafetyManager::get_state() == SAFETY_EMERGENCY_STOP) {
        MotorControl::hard_stop();
        return;
    }

    /* --- Speed Constant --- */
    // Convert per-mille (0-1000) to float (0.0-1.0)
    constexpr float speed = (float)AUTO_SPEED / 1000.0f;
    
    /* --- State Machine --- */
    switch (state) {

        /* ---------------- MOVING ---------------- */
        case AutoState::MOVING: {
            if (ObstacleDetection::get_front().in_stop_zone) {
                MotorControl::hard_stop();
                DirectionalScan::start_sweep();
                enter(AutoState::SCANNING);
            } else {
                MotorControl::apply_command({ speed, 0.0f, 0.0f });
            }
            break;
        }

        /* ---------------- SCANNING ---------------- */
        case AutoState::SCANNING: {
            DirectionalScan::update_sweep();
            if (DirectionalScan::sweep_ready()) {
                spin_limit_ms = pick_best_spin_time(DirectionalScan::get_sweep_result(), spin_direction);

                if (spin_limit_ms > 0) {
                    // Center the sensor for the next MOVING state
                    Sensors::scan_set_direction(ScanDir::FRONT);
                    enter(AutoState::SPINNING);
                } else {
                    // No forward path: check if rear escape is possible
                    if (Sensors::get_rear_distance_raw_cm() > REAR_SLOW_EXIT_CM) {
                        enter(AutoState::BACKING_UP);
                    } else {
                        enter(AutoState::STUCK);
                    }
                }
            }
            break;
        }

        /* ---------------- SPINNING ---------------- */
        case AutoState::SPINNING: {
            // Check if hardcoded rotation duration has elapsed
            if (millis() - timer_ms >= (unsigned long)spin_limit_ms) {
                MotorControl::hard_stop();
                enter(AutoState::MOVING);
            } else {
                // Fixed duration rotation at autonomous speed
                MotorControl::apply_command({ 0.0f, 0.0f, (speed * spin_direction) });
            }
            break;
        }

        /* ---------------- BACKING_UP ---------------- */
        case AutoState::BACKING_UP: {
            // Reverse escape (max 1s or until rear is blocked)
            if (millis() - timer_ms >= 1000 || Sensors::get_rear_distance_raw_cm() <= REAR_STOP_ENTER_CM) {
                MotorControl::hard_stop();
                DirectionalScan::start_sweep(); 
                enter(AutoState::SCANNING);
            } else {
                MotorControl::apply_command({ -speed, 0.0f, 0.0f });
            }
            break;
        }

        /* ---------------- STUCK ---------------- */
        case AutoState::STUCK: {
            MotorControl::hard_stop();
            if (millis() - timer_ms >= AUTO_RETRY_WAIT_MS) {
                DirectionalScan::start_sweep();
                enter(AutoState::SCANNING);
            }
            break;
        }
    }
}

} // namespace AutonomousController