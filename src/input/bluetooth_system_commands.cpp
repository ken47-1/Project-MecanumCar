/* ==================== bluetooth_system_commands.cpp ==================== */
#include "input/bluetooth_system_commands.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"
#include "log/log.h"

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
static bool awaiting_log_cmd              = false;
static bool arc_turn_speed_dependent_flag = ARC_TURN_DEFAULT_MODE;

/* =============== INTERNAL HELPERS =============== */
/* ============ LOGIC ============ */
static void toggle_ch(Log::Ch c) {
    Log::setChannel(c, !Log::isChannelEnabled(c));
}

/* =============== PUBLIC API =============== */
/* ============ STATE ============ */
bool arc_turn_speed_dependent() {
    return arc_turn_speed_dependent_flag;
}

void set_arc_turn_speed_dependent(bool value) {
    arc_turn_speed_dependent_flag = value;
}

bool handle_char(char c, InputWatchdog& watchdog) {
    /* ============ LOG TOGGLE ============ */
    /* Intercept first. 'GP' must not fall through to the 'P' PID toggle. */
    if (awaiting_log_cmd) {
        awaiting_log_cmd = false;
        bool verbose = false;

        switch (c) {
            case 'C': toggle_ch(Log::Ch::CH_COM);  break;
            case 'I': toggle_ch(Log::Ch::CH_INP);  break;
            case 'M': toggle_ch(Log::Ch::CH_MOT);  break;
            case 'R': toggle_ch(Log::Ch::CH_RMP);  break;
            case 'P': toggle_ch(Log::Ch::CH_PID);  break;
            case 'S': toggle_ch(Log::Ch::CH_SNR);  break;
            case 'F': toggle_ch(Log::Ch::CH_SAF);  break;
            case 'W': toggle_ch(Log::Ch::CH_WDG);  break;
            case 'G': verbose = true;              break;
            case '+': Log::setAllChannels(true);   break;
            case '-': Log::setAllChannels(false);  break;
            default: break;
        }
        if (verbose) {
            Log::dump();
        } else {
            Log::dumpShort();
        }
        watchdog.feed();
        return true;
    }

    switch (c) {
        /* ============ LOG ============ */
        case 'G':
            awaiting_log_cmd = true;
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