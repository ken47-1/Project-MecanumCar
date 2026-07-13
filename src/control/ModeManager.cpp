/* ==================== ModeManager.cpp ==================== */
#include "control/ModeManager.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "comms/Comms.h"
#include "control/MotorControl.h"
#include "control/AutonomousController.h"
#include "sensors/DirectionalScan.h"

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
