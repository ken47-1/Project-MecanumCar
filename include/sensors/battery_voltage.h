/* ==================== battery_voltage.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ CORE ============ */
#include <stdint.h>

#if ENABLE_BATTERY_MONITOR

/* =============== API =============== */
namespace BatteryVoltage {
    float get_voltage();
    bool is_low();
    void report();
}

#else

/* =============== API =============== */
namespace BatteryVoltage {
    inline float get_voltage() { return BATTERY_VOLTAGE_MAX; }
    inline bool is_low() { return false; }
    inline void report() {}
}

#endif // ENABLE_BATTERY_MONITOR