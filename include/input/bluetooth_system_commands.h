/* ==================== bluetooth_system_commands.h ==================== */
#pragma once

/* ==================== FORWARD DECLARATIONS ==================== */
class InputWatchdog;

/* =============== API =============== */
namespace BluetoothSystemCommands {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, InputWatchdog& watchdog);
}
