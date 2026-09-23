/* ==================== bluetooth_command_parser.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ FORWARD DECLS ============ */
class InputWatchdog;

/* =============== API =============== */
namespace BluetoothCommandParser {
    void handle(InputWatchdog& watchdog);
}
