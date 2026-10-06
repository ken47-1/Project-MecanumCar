/* ==================== bluetooth_command_parser.cpp ==================== */
#include "input/bluetooth_command_parser.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/mode_manager.h"
#include "control/motion_command.h"
#include "control/motor_control.h"

/* ========= INPUT ========= */
#include "input/input_watchdog.h"
#include "input/bluetooth_system_commands.h"
#include "input/bluetooth_button_input.h"
#include "input/bluetooth_speed_authority.h"

/* ========= SENSORS ========= */
#include "sensors/directional_scan.h"

namespace BluetoothCommandParser {

/* =============== PUBLIC API =============== */
void handle(InputWatchdog& watchdog) {
    MotionCommand cmd = {0.0f, 0.0f, 0.0f};
    bool valid_input    = false;
    bool motion_applied = false;
    bool explicit_stop  = false;

    /* --- Parsing Loop --- */
    while (Comms::available()) {
        char c = (char)Comms::read();
        if (c == '\n' || c == '\r' || c == ' ') continue;

        /* --- System Commands --- */
        if (BluetoothSystemCommands::handle_char(c, watchdog)) {
            if (c == '!') {
                return;
            }
            
            if (c == 'X') {
                explicit_stop = true;
            }

            valid_input = true;
            continue;
        }

        /* --- Speed Control --- */
        if (BluetoothSpeedAuthority::handle_char(c)) {
            valid_input = true;
            continue;
        }

        /* --- Button Input --- */
        if (BluetoothButtonInput::handle_char(c, cmd)) {
            motion_applied = true;
            valid_input    = true;
            continue;
        }
    }

    /* --- ARC TURNING --- */
    if (BluetoothSystemCommands::arc_turn_speed_dependent()) {
        float speed_factor = fabs(cmd.forward);
        float rotate_scale = SD_MAX_SCALE - (SD_MAX_SCALE - SD_MIN_SCALE) * speed_factor;
        cmd.rotate *= rotate_scale;
    } else {
        cmd.rotate *= FIXED_ROTATE_SCALE;
    }

    /* --- Autonomous Exit on Manual Input --- */
    if (ModeManager::is_autonomous() && (explicit_stop || motion_applied)) {
        ModeManager::set(DriveMode::MANUAL);
    }

    /* --- Execution --- */
    if (!ModeManager::is_autonomous()) {

        /* --- Priority 1: Explicit Stop --- */
        if (explicit_stop) {
            MotorControl::apply_command({0.0f, 0.0f, 0.0f});
            return;
        }
        /* --- Priority 2: Direct Manual Control --- */
        else if (motion_applied) {
            DirectionalScan::update(cmd);
            MotorControl::apply_command(cmd);
        }
    }

    /* --- Watchdog (Standard Feed) --- */
    if (valid_input) {
        watchdog.feed();
    }
}

} // namespace BluetoothCommandParser
