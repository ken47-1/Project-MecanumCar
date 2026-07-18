/* ==================== bluetooth_system_commands.cpp ==================== */
#include "input/bluetooth_system_commands.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/motor_fault.h"
#include "input/input_watchdog.h"
#include "control/mode_manager.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== PUBLIC API =============== */
namespace BluetoothSystemCommands {

bool handle_char(char c, InputWatchdog& watchdog) {
    switch (c) {
        /* ============ SAFETY & WATCHDOG ============ */
        case '!':
            MotorFault::trigger(MotorFaultReason::ESTOP);
            watchdog.feed(); // Action feeds watchdog
            return true;

        case '?':
            MotorFault::reset();
            return true;

        case '^':
            /* THE MASTER KEY: Explicitly feeds the watchdog */
            watchdog.feed();
            return true;

        case 'X':
            /* HEARTBEAT: Standard idle signal feeds watchdog */
            watchdog.feed();
            return true;

        /* ============ DRIVE MODES ============ */
        case '1':
            ModeManager::set(DriveMode::AUTONOMOUS);
            watchdog.feed(); // Mode switch feeds watchdog
            return true;

        case '0':
            ModeManager::set(DriveMode::MANUAL);
            watchdog.feed(); // Mode switch feeds watchdog
            return true;

        default:
            return false;
    }
}

} // namespace BluetoothSystemCommands