/* ==================== BluetoothSpeedAuthority.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <stdint.h>

/* =============== API =============== */
namespace BluetoothSpeedAuthority {

    // Handle a single incoming Bluetooth character.
    // Returns true if the character was consumed. 
    bool handle_char(char c);

    //Get current speed authority as PERCENT (0–100).
    float get_speed_scale();

}