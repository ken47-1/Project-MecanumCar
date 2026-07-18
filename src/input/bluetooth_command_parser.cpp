/* ==================== bluetooth_command_parser.cpp ==================== */
#include "input/bluetooth_command_parser.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/comms.h"
#include "control/mode_manager.h"
#include "control/motion_command.h"
#include "control/motor_control.h"
#include "input/input_watchdog.h"
#include "input/bluetooth_system_commands.h"
#include "input/bluetooth_button_input.h"
#include "input/bluetooth_speed_authority.h"
#include "sensors/directional_scan.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== PUBLIC API =============== */
namespace BluetoothCommandParser {

/* =============== INTERNAL STATE =============== */
/* ============ TIMED ACTIONS ============ */
static unsigned long manual_spin_start_ms = 0;
static uint16_t      manual_spin_limit_ms = 0;
static float         manual_spin_dir      = 0.0f;
static bool          manual_spin_active   = false;

/* =============== PUBLIC API =============== */
void handle(InputWatchdog& watchdog) {
    MotionCommand cmd = {0.0f, 0.0f, 0.0f};
    bool valid_input    = false;
    bool motion_applied = false;
    bool explicit_stop  = false;

    /* --------- Parsing Loop --------- */
    while (Comms::available()) {
        char c = (char)Comms::read();
        if (c == '\n' || c == '\r' || c == ' ') continue;

        /* --- System Commands --- */
        if (BluetoothSystemCommands::handle_char(c, watchdog)) {
            if (c == '!') return; 
            if (c == 'X') explicit_stop = true;
            valid_input = true;
            continue;
        }

        
        /* --- Timed Spin Triggers --- */
        // SPEED: 600
        /*
        if (c == '[' || c == ']' || c == '{' || c == '}') {
            manual_spin_active = true;
            manual_spin_start_ms = millis();
            manual_spin_dir = (c == '[' || c == '{') ? -1.0f : 1.0f;
            manual_spin_limit_ms = (c == '[' || c == ']') ? AUTO_SPIN_DIAGONAL_MS : AUTO_SPIN_SIDE_MS;
            
            if (ModeManager::is_autonomous()) ModeManager::set(DriveMode::MANUAL);
            valid_input = true;
            continue;
        }
        */

        /* --- Speed Control --- */
        if (BluetoothSpeedAuthority::handle_char(c)) {
            valid_input = true;
            continue;
        }

        /* --- Manual Motion Input --- */
        if (BluetoothButtonInput::handle_char(c, cmd)) {
            
            /* Manual Override: Cancel timed actions if joystick/buttons move */
            manual_spin_active = false;

            if (ModeManager::is_autonomous()) {
                ModeManager::set(DriveMode::MANUAL);
            }
            motion_applied = true;
            valid_input    = true;
        }
    }

    /* --------- Execution --------- */
    if (!ModeManager::is_autonomous()) {
        
        /* --- Priority 1: Direct Manual Control --- */
        if (motion_applied) {
            DirectionalScan::update(cmd);
            MotorControl::apply_command(cmd);
        } 
        /* --- Priority 2: Timed Spin Action --- */
        else if (manual_spin_active) {
            
            /* --- Timed Authority --- */
            // Feed the watchdog while the timed spin is in progress
            watchdog.feed();

            if (millis() - manual_spin_start_ms >= manual_spin_limit_ms) {
                manual_spin_active = false;
                MotorControl::apply_command({0.0f, 0.0f, 0.0f});
            } else {
                // Use AUTO_SPEED for turns to ensure calculated accuracy
                float spd = (float)AUTO_SPEED / 1000.0f;
                MotorControl::apply_command({0.0f, 0.0f, (spd * manual_spin_dir)});
            }
        }
        /* --- Priority 3: Explicit Stop --- */
        else if (explicit_stop) {
            MotorControl::apply_command({0.0f, 0.0f, 0.0f});
        }
    }

    /* --------- Watchdog (Standard Feed) --------- */
    if (valid_input) {
        watchdog.feed();
    }
}

} // namespace BluetoothCommandParser