/* ==================== bluetooth_system_commands.h ==================== */
#pragma once

/* ==================== FORWARD DECLARATIONS ==================== */
class InputWatchdog;

/* =============== INTERNAL STATE =============== */
extern bool arc_turn_speed_dependent;

/* =============== API =============== */
namespace BluetoothSystemCommands {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, InputWatchdog& watchdog);
}
