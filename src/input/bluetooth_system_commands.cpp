/* ==================== bluetooth_system_commands.cpp ==================== */
#include "config/Config.h"
#include "input/bluetooth_system_commands.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/motor_fault.h"
#include "safety/safety_manager.h"
#include "input/input_watchdog.h"
#include "control/mode_manager.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
bool arc_turn_speed_dependent = ARC_TURN_DEFAULT_MODE;

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
            SafetyManager::clear_emergency_stop();
            return true;

        case '^':
            /* THE MASTER KEY: Explicitly feeds the watchdog */
            watchdog.feed();
            return true;

        case 'X':
            /* HEARTBEAT: Standard idle signal feeds watchdog */
            watchdog.feed();
            return true;

        /* ============ ARC TURN TOGGLE ============ */
        case 'T':
            arc_turn_speed_dependent = !arc_turn_speed_dependent;
            Comms::system.print("Arc turn: ");
            Comms::system.println(arc_turn_speed_dependent ? "Speed-Dependent" : "Fixed");
            watchdog.feed();
            return true;            

        /* ============ DRIVE MODES ============ */
        case '1':
            #if ENABLE_AUTONOMOUS_MODE
                ModeManager::set(DriveMode::AUTONOMOUS);
                watchdog.feed();
                return true;
            #else
                Comms::system.println("ERROR: Autonomous mode not compiled");
                return false;
            #endif

        case '0':
            ModeManager::set(DriveMode::MANUAL);
            watchdog.feed();
            return true;

        default:
            return false;
    }
}

} // namespace BluetoothSystemCommands