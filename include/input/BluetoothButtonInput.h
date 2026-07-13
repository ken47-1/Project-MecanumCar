/* ==================== BluetoothButtonInput.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/MotionCommand.h"

/* =============== API =============== */
namespace BluetoothButtonInput {
    // Returns true if the character was a system command and consumed
    bool handle_char(char c, MotionCommand& out);
}