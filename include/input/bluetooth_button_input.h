/* ==================== bluetooth_button_input.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */
#include "control/motion_command.h"

/* =============== API =============== */
#if ENABLE_INPUT_BUTTONS
namespace BluetoothButtonInput {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, MotionCommand& out);
}
#else
namespace BluetoothButtonInput {
    inline bool handle_char(char, MotionCommand&) { return false; }
}
#endif