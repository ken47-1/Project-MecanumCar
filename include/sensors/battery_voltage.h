/* ==================== battery_voltage.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <stdint.h>

/* =============== API =============== */
namespace BatteryVoltage {
    /* --------- Telemetry --------- */
    float get_voltage();
    bool is_low();
    void report();
}