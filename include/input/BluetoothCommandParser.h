/* ==================== BluetoothCommandParser.h ==================== */
#pragma once

/* ==================== FORWARD DECLARATIONS ==================== */
class InputWatchdog;

/* =============== API =============== */
namespace BluetoothCommandParser {
    // Returns true if the character was a system command and consumed
    void handle(InputWatchdog& watchdog);
}