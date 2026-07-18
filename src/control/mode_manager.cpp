/* ==================== mode_manager.cpp ==================== */
#include "control/mode_manager.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "comms/comms.h"
#include "control/motor_control.h"
#include "control/autonomous_controller.h"
#include "sensors/directional_scan.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace ModeManager {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static DriveMode current_mode = DriveMode::MANUAL;

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    current_mode = DriveMode::MANUAL;
    Comms::system.println("ModeManager INIT");
}

/* ============ STATE ============ */
void set(DriveMode mode) {
    if (mode == current_mode) return;

    current_mode = mode;

    MotorControl::hard_stop();
    DirectionalScan::reset();

    if (mode == DriveMode::AUTONOMOUS) {
        AutonomousController::reset();
        Comms::system.println("Mode: AUTONOMOUS");
    } else {
        Comms::system.println("Mode: MANUAL");
    }
}

DriveMode get() {
    return current_mode;
}

bool is_autonomous() {
    return current_mode == DriveMode::AUTONOMOUS;
}

} // namespace ModeManager
