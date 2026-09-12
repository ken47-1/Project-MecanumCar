/* ==================== bluetooth_system_commands.cpp ==================== */
#include "input/bluetooth_system_commands.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/motor_fault.h"
#include "control/mode_manager.h"
#if ENABLE_ENCODERS
    #include "control/motor_pid.h"
#endif

/* ========= SAFETY ========= */
#include "safety/safety_manager.h"

/* ========= INPUT ========= */
#include "input/input_watchdog.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace BluetoothSystemCommands {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static bool arc_turn_speed_dependent_flag = ARC_TURN_DEFAULT_MODE;

/* =============== PUBLIC API =============== */
/* ============ STATE ============ */
bool arc_turn_speed_dependent() {
    return arc_turn_speed_dependent_flag;
}

void set_arc_turn_speed_dependent(bool value) {
    arc_turn_speed_dependent_flag = value;
}    

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
            arc_turn_speed_dependent_flag = !arc_turn_speed_dependent_flag;
            Comms::system.print("Arc turn: ");
            Comms::system.println(arc_turn_speed_dependent() ? "Speed-Dependent" : "Fixed");
            watchdog.feed();
            return true;

        /* ============ PID TOGGLE ============ */
        case 'P':
            #if ENABLE_ENCODERS
                MotorPID::set_closed_loop(!MotorPID::is_closed_loop());
                Comms::system.print("PID: ");
                Comms::system.println(MotorPID::is_closed_loop() ? "closed" : "open");
                watchdog.feed();
                return true;
            #else
                Comms::system.println("ERROR: Encoders not compiled");
                return false;
            #endif

        /* ============ DRIVE MODES ============ */
        case '1':
            #if ENABLE_AUTONOMOUS_MODE
                ModeManager::set(DriveMode::AUTONOMOUS);
                watchdog.feed();
                return true;
            #else
                Comms::system.println("ERROR: Autonomous mode not compiled");
                watchdog.feed();
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
