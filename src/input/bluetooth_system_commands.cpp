/* ==================== bluetooth_system_commands.cpp ==================== */
#include "input/bluetooth_system_commands.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"
#include "comms/debug.h"

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
static bool awaiting_debug_cmd            = false;
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
    /* ============ DEBUG TOGGLE ============ */
    /* Intercept first. 'GP' must not fall through to the 'P' PID toggle. */
    if (awaiting_debug_cmd) {
        awaiting_debug_cmd = false;
        bool verbose = false;

        switch (c) {
            case 'C': Debug::toggle(Debug::Ch::COMMS);    break;
            case 'I': Debug::toggle(Debug::Ch::IN);       break;
            case 'M': Debug::toggle(Debug::Ch::MOTOR);    break;
            case 'R': Debug::toggle(Debug::Ch::RAMP);     break;
            case 'P': Debug::toggle(Debug::Ch::PID);      break;
            case 'S': Debug::toggle(Debug::Ch::SENSORS);  break;
            case 'F': Debug::toggle(Debug::Ch::SAFETY);   break;
            case 'W': Debug::toggle(Debug::Ch::WATCHDOG); break;
            case 'G': verbose = true;                     break;
            case '+': Debug::set_all(true);               break;
            case '-': Debug::set_all(false);              break;
            default: break;
        }
        if (verbose) {
            Debug::dump();
        } else {
            Debug::dump_short();
        }
        watchdog.feed();
        return true;
    }

    switch (c) {
        /* ============ DEBUG ============ */
        case 'G':
            awaiting_debug_cmd = true;
            return true;

        /* ============ SAFETY & WATCHDOG ============ */
        case '!':
            MotorFault::trigger(MotorFaultReason::ESTOP);
            watchdog.feed();
            return true;

        case '?':
            SafetyManager::clear_emergency_stop();
            return true;

        case '^':
            watchdog.feed();
            return true;

        case 'X':
            watchdog.feed();
            return true;

        /* ============ ARC TURN TOGGLE ============ */
        case 'T':
            arc_turn_speed_dependent_flag = !arc_turn_speed_dependent_flag;
            Comms::system.print(F("Arc turn: "));
            Comms::system.println(
                arc_turn_speed_dependent()
                    ? F("Speed-Dependent")
                    : F("Fixed"));
            watchdog.feed();
            return true;

        /* ============ PID TOGGLE ============ */
        case 'P':
            #if ENABLE_ENCODERS
                MotorPID::set_closed_loop(!MotorPID::is_closed_loop());
                Comms::system.print(F("PID: "));
                Comms::system.println(
                    MotorPID::is_closed_loop() ? F("closed") : F("open"));
                watchdog.feed();
                return true;
            #else
                Comms::system.println(F("ERROR: Encoders not compiled"));
                return false;
            #endif

        /* ============ PID TELEMETRY ============ */
        case 'K':
            #if ENABLE_ENCODERS
            {
                Comms::system.println(F("==================="));
                Comms::system.println(F("[PID] K STATUS"));

                char buf[64];
                snprintf(buf, sizeof(buf), "[PID] KP=%.2f KI=%.2f KD=%.2f MODE=%s",
                         (double)PID_KP, (double)PID_KI, (double)PID_KD,
                         MotorPID::is_closed_loop() ? "CLOSED" : "OPEN");
                Comms::system.println(buf);

                const char* names[4] = { "FL", "FR", "RL", "RR" };
                for (uint8_t i = 0; i < 4; i++) {
                    WheelPID w = MotorPID::get_wheel(i);
                    snprintf(buf, sizeof(buf), "[PID] %s T=%.2f M=%.2f E=%+.2f O=%.2f",
                             names[i], (double)w.target, (double)w.measured, (double)w.error, (double)w.output);
                    Comms::system.println(buf);
                }

                Comms::system.println(F("==================="));
                watchdog.feed();
                return true;
            }
            #else
                Comms::system.println(F("ERROR: Encoders not compiled"));
                return false;
            #endif

        /* ============ DRIVE MODES ============ */
        case '1':
            #if ENABLE_AUTONOMOUS_MODE
                ModeManager::set(DriveMode::AUTONOMOUS);
                watchdog.feed();
                return true;
            #else
                Comms::system.println(F("ERROR: Autonomous mode not compiled"));
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