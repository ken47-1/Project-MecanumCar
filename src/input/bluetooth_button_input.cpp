/* ==================== bluetooth_button_input.cpp ==================== */
#include "input/bluetooth_button_input.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== PUBLIC API =============== */
namespace BluetoothButtonInput {

bool handle_char(char c, MotionCommand& out) {
#if !ENABLE_INPUT_BUTTONS
    return false;
#endif

    switch (c) {
        // Forward / Backward
        case 'W': out.forward += 1.0f; break;
        case 'S': out.forward -= 1.0f; break;

        // Strafes
        case 'A': out.strafe  -= 1.0f; break;
        case 'D': out.strafe  += 1.0f; break;

        // Diagonals
        case 'Q': out.forward += 1.0f; out.strafe -= 1.0f; break;
        case 'E': out.forward += 1.0f; out.strafe += 1.0f; break;
        case 'Z': out.forward -= 1.0f; out.strafe -= 1.0f; break;
        case 'C': out.forward -= 1.0f; out.strafe += 1.0f; break;

        // Rotation
        case 'J': out.rotate  -= 1.0f; break;
        case 'L': out.rotate  += 1.0f; break;

        default: return false;
    }
    return true;
}

} // namespace BluetoothButtonInput