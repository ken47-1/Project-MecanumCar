/* ==================== bluetooth_button_input.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/motion_command.h"

/* =============== API =============== */
namespace BluetoothButtonInput {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, MotionCommand& out);
}