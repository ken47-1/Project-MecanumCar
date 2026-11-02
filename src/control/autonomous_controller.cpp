/* ==================== autonomous_controller.cpp ==================== */
#include "control/autonomous_controller.h"

#if ENABLE_AUTONOMOUS_MODE

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/motion_command.h"
#include "control/motor_control.h"
#include "control/mode_manager.h"

/* ========= SAFETY ========= */
#include "safety/obstacle_detection.h"
#include "safety/safety_manager.h"
#include "safety/system_fault.h"

/* ========= SENSORS ========= */
#include "sensors/directional_scan.h"
#include "sensors/ultrasonic.h"
#include "sensors/encoder.h"

/* ========= INPUT ========= */
#include "input/input_watchdog.h"

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
static float         last_spin_dir  = 0.0f;
static int32_t       target_ticks   = 0;
static int32_t       start_ticks    = 0;
static uint16_t      spin_limit_ms  = 0;
static unsigned long timer_ms       = 0;
static uint8_t       stuck_retries  = 0;

/* =============== INTERNAL HELPERS =============== */
/* ============ KINEMATICS ============ */
#if ENABLE_ENCODERS
    constexpr float WHEEL_CIRC_MM = PI * WHEEL_DIAMETER_MM;
    constexpr float MM_PER_TICK   = WHEEL_CIRC_MM / ENCODER_TICKS_PER_REV;
    constexpr float TRACK_CIRC_MM = PI * TRACK_WIDTH_MM;

    constexpr int32_t TICKS_DIAGONAL = (TRACK_CIRC_MM * (45.0f / 360.0f)) / MM_PER_TICK;
    constexpr int32_t TICKS_SIDE     = (TRACK_CIRC_MM * (90.0f / 360.0f)) / MM_PER_TICK;
#endif

/* ============ NAVIGATION ============ */
static void enter(AutoState next) {
    state = next;
    timer_ms = millis();
}

/* Direct spin command. Bypasses MotorRamp so the burst duration equals actual rotation time. */
static void spin_direct(float direction, float magnitude) {
    MotorControl::apply_command_instant({0.0f, 0.0f, magnitude * direction});
}

/* --- Search all swept directions for the absolute widest path --- */
static int32_t pick_best_spin_ticks(const SweepResult& r, float& dir_out, uint16_t& time_out) {
    
    struct PathOption {
        uint16_t dist;
        float    direction; // -1.0 Left, 1.0 Right
        int32_t  ticks;     // Tick target to reach this angle
        uint16_t time_ms;   // Time target if encoders are disabled
    };

    /* Define turn candidates with their specific requirements */
    PathOption candidates[] = {
        { r.front_left,  -1.0f, 0, AUTO_SPIN_DIAGONAL_MS }, // 45 deg
        { r.front_right,  1.0f, 0, AUTO_SPIN_DIAGONAL_MS }, // 45 deg
        { r.left,        -1.0f, 0, AUTO_SPIN_SIDE_MS     }, // 90 deg
        { r.right,        1.0f, 0, AUTO_SPIN_SIDE_MS     }  // 90 deg
    };

    #if ENABLE_ENCODERS
        candidates[0].ticks = TICKS_DIAGONAL;
        candidates[1].ticks = TICKS_DIAGONAL;
        candidates[2].ticks = TICKS_SIDE;
        candidates[3].ticks = TICKS_SIDE;
    #endif

    uint16_t max_dist = 0;
    uint8_t  best_idx[4] = {0, 0, 0, 0};
    uint8_t  best_n = 0;

    /* --- Absolute Best Search --- */
    for (uint8_t i = 0; i < 4; i++) {
        if (candidates[i].dist > max_dist) {
            max_dist = candidates[i].dist;
            best_n = 1;
            best_idx[0] = i;
        } else if (candidates[i].dist == max_dist && best_n < 4) {
            best_idx[best_n++] = i;
        }
    }

    /* --- Path Clearance Check --- */
    if (max_dist < FRONT_SLOW_EXIT_CM) {
        dir_out = 0.0f;
        return 0;
    }

    /* --- Alternating Tiebreaker --- */
    uint8_t pick = best_idx[0];
    if (best_n > 1) {
        for (uint8_t i = 0; i < best_n; i++) {
            if (candidates[best_idx[i]].direction != last_spin_dir) {
                pick = best_idx[i];
                break;
            }
        }
    }
    last_spin_dir = candidates[pick].direction;
    dir_out     = candidates[pick].direction;
    time_out    = candidates[pick].time_ms;
    
    return (candidates[pick].ticks > 0) ? candidates[pick].ticks : 1; // Return >0 to indicate path found
}

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void reset() {
    state = AutoState::MOVING;
    stuck_retries = 0;
    DirectionalScan::reset();
}

/* ============ LOGIC LOOP ============ */
void update(InputWatchdog& watchdog) {
    /* --- Mode Guard --- */
    if (!ModeManager::is_autonomous()) {
        return;
    }

    /* --- Automatic Authority --- */
    watchdog.feed();

    /* --- Global Safety Check --- */
    if (SafetyManager::get_state() == SAFETY_EMERGENCY_STOP) {
        MotorControl::hard_stop();
        return;
    }

    /* --- Speed Constants --- */
    constexpr float speed = (float)AUTO_SPEED / 1000.0f;
    
    /* --- State Machine --- */
    switch (state) {

        /* ------ MOVING ------ */
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

        /* ------ SCANNING ------ */
        case AutoState::SCANNING: {
            DirectionalScan::update_sweep();
            
            if (DirectionalScan::sweep_ready()) {
                target_ticks = pick_best_spin_ticks(DirectionalScan::consume_sweep_result(), spin_direction, spin_limit_ms);

                if (target_ticks > 0) {
                    stuck_retries = 0; // Path found, reset counter
                    Ultrasonic::scan_set_direction(ScanDir::FRONT);
                    #if ENABLE_ENCODERS
                        start_ticks = Encoder::get_count(0); // Snapshot FL wheel
                    #endif
                    enter(AutoState::SPINNING);
                } else {
                    stuck_retries++; // Failed to find a path
                    
                    if (stuck_retries >= STUCK_MAX_RETRIES) {
                        enter(AutoState::STUCK);
                    } else {
                        // No forward path: check if rear escape is possible
                        #if ENABLE_ULTRASONIC_REAR
                            if (Ultrasonic::get_rear_distance_raw_cm() > REAR_STOP_EXIT_CM) {
                                enter(AutoState::BACKING_UP);
                            } else {
                                enter(AutoState::STUCK);
                            }
                        #else
                            enter(AutoState::STUCK);
                        #endif
                    }
                }
            } 
            /* --- Sweep Timeout Safety --- */
            else if (millis() - timer_ms > AUTO_SWEEP_TIMEOUT_MS) {
                SystemFault::trigger(SystemFaultReason::SENSOR_FAIL);
                MotorControl::hard_stop();
                enter(AutoState::STUCK);
            }
            break;
        }

        /* ------ SPINNING ------ */
        case AutoState::SPINNING: {
            bool spin_done = false;

            #if ENABLE_ENCODERS
                int32_t current_ticks = Encoder::get_count(0);
                int32_t traveled = abs(current_ticks - start_ticks);
                if (traveled >= target_ticks) spin_done = true;
            #else
                if (millis() - timer_ms >= (unsigned long)spin_limit_ms) spin_done = true;
            #endif

            if (spin_done) {
                MotorControl::hard_stop();
                enter(AutoState::MOVING);
            } else {
                // Spin at a capped speed to reduce kinetic energy
                constexpr float spin_speed = (float)AUTO_SPIN_SPEED / 1000.0f;
                spin_direct(spin_direction, spin_speed);
            }
            break;
        }

        /* ------ BACKING_UP ------ */
        case AutoState::BACKING_UP: {
            bool rear_blocked = false;
            #if ENABLE_ULTRASONIC_REAR
                rear_blocked = (Ultrasonic::get_rear_distance_raw_cm() <= REAR_STOP_ENTER_CM);
            #else
                rear_blocked = false;  // assume clear
            #endif
            
            if (millis() - timer_ms >= AUTO_BACKUP_MS || rear_blocked) {
                MotorControl::hard_stop();
                DirectionalScan::start_sweep();
                enter(AutoState::SCANNING);
            } else {
                constexpr float rev = (float)AUTO_REVERSE_SPEED / 1000.0f;
                MotorControl::apply_command({ -rev, 0.0f, 0.0f });
            }
            break;
        }

        /* ------ STUCK ------ */
        case AutoState::STUCK: {
            MotorControl::hard_stop();

            if (stuck_retries >= STUCK_MAX_RETRIES) {
                break;  // Give up. Wait for user intervention.
            }

            if (millis() - timer_ms >= AUTO_RETRY_WAIT_MS) {
                stuck_retries++;
                DirectionalScan::start_sweep();
                enter(AutoState::SCANNING);
            }
            break;
        }
    }
}

} // namespace AutonomousController

#endif // ENABLE_AUTONOMOUS_MODE