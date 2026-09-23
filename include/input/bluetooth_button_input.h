/* ==================== bluetooth_button_input.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= CONTROL ========= */
#include "control/motion_command.h"

#if ENABLE_INPUT_BUTTONS

/* =============== API =============== */
namespace BluetoothButtonInput {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, MotionCommand& out);
}

#else

namespace BluetoothButtonInput {
    inline bool handle_char(char, MotionCommand&) { return false; }
}

#endif // ENABLE_INPUT_BUTTONS
