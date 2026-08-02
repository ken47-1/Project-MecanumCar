/* ==================== mode_manager.cpp ==================== */
#include "config/Config.h"
#include "control/mode_manager.h"

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */
#include "comms/comms.h"
#include "control/motor_control.h"
#if ENABLE_AUTONOMOUS_MODE
    #include "control/autonomous_controller.h"
#endif
#if ENABLE_DIRECTIONAL_SCAN
    #include "sensors/directional_scan.h"
#endif

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
    if (mode == current_mode) {
        return;
    }

    current_mode = mode;

    MotorControl::hard_stop();
    #if ENABLE_DIRECTIONAL_SCAN
        DirectionalScan::reset();
    #endif

    if (mode == DriveMode::AUTONOMOUS) {
        #if ENABLE_AUTONOMOUS_MODE
            AutonomousController::reset();
        #endif
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
