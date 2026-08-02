/* ==================== battery_voltage.cpp ==================== */
#include "config/Config.h"
#include "config/HardwareConfig.h"

#if ENABLE_BATTERY_MONITOR

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */
#include "sensors/battery_voltage.h"
#include "safety/safety_manager.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace BatteryVoltage {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static float filtered_voltage = 0.0f;
static bool initialized = false;

/* =============== INTERNAL HELPERS =============== */
/* ============ FILTERING ============ */
static float read_voltage() {
    int raw = analogRead(BATTERY_SENSOR_PIN);
    float v = (raw / 1023.0f) * 5.0f * BATTERY_DIVIDER_RATIO;
    
    if (!initialized) {
        filtered_voltage = v;
        initialized = true;
    } else {
        filtered_voltage += BATTERY_EMA_ALPHA * (v - filtered_voltage);
    }
    
    return filtered_voltage;
}

/* =============== PUBLIC API =============== */
/* ============ TELEMETRY ============ */
float get_voltage() {
    return read_voltage();
}

bool is_low() {
    return get_voltage() < BATTERY_WARNING_VOLTAGE;
}

void report() {
    static unsigned long last_report = 0;
    if (millis() - last_report >= BATTERY_REPORT_INTERVAL_MS) {

        float v   = get_voltage();
        float min = SafetyManager::get_min_voltage();

        char buf[20];

        /* Filtered Voltage */
        snprintf(buf, sizeof(buf), "*V%.2fV*", v);
        Comms::print.println(buf);

        /* Minimum Voltage */
        snprintf(buf, sizeof(buf), "*M%.2fV*", min);
        Comms::print.println(buf);
    
        last_report = millis();
    }
}

} // namespace BatteryVoltage

#endif // ENABLE_BATTERY_MONITOR