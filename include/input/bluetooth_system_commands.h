/* ==================== bluetooth_system_commands.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ FORWARD DECLS ============ */
class InputWatchdog;

/* =============== API =============== */
namespace BluetoothSystemCommands {
    /* ============ State ============ */
    bool arc_turn_speed_dependent();
    void set_arc_turn_speed_dependent(bool value);

    /* ============ Input ============ */
    bool handle_char(char c, InputWatchdog& watchdog);
}
