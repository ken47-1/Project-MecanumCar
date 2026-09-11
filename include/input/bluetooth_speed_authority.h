/* ==================== bluetooth_speed_authority.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ CORE ============ */
#include <stdint.h>

#if ENABLE_INPUT_SPEED_AUTHORITY

/* =============== API =============== */
namespace BluetoothSpeedAuthority {
    bool handle_char(char c);
    float get_speed_scale();
}

#else

/* =============== API =============== */
namespace BluetoothSpeedAuthority {
    inline bool handle_char(char) { return false; }
    inline float get_speed_scale() { return 1.0f; }
}

#endif