/* ==================== safety_manager.cpp ==================== */
#include "safety/safety_manager.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"
#include "config/HardwareConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"
#include "log/log.h"

/* ========= CONTROL ========= */
#include "control/motor_fault.h"

/* ========= SENSORS ========= */
#include "sensors/ultrasonic.h"
#include "sensors/directional_scan.h"
#include "sensors/battery_voltage.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace SafetyManager {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static SafetyState current_state   = SAFETY_CLEAR;
static unsigned long last_decay_ms = 0;
static bool emergency_stop_latched = false;
static bool input_loss_active      = false;
static bool connection_loss_active = false;

/* ============ BATTERY TRACKING ============ */
static float min_voltage_seen = 10.0f;
static unsigned long last_warning_ms = 0;
static unsigned long last_critical_ms = 0;

/* ============ ESTOP EDGE ============ */
static bool last_estop = false;

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    current_state = SAFETY_CLEAR;
    last_decay_ms = millis();
    emergency_stop_latched = false;
    input_loss_active = false;
    connection_loss_active = false;
    min_voltage_seen = 10.0f;
    last_warning_ms = 0;
    last_critical_ms = 0;
    Comms::system.println(F("SafetyManager INIT"));
}

/* ============ LOGIC ============ */
void update() {
    bool estop_active = emergency_stop_latched || MotorFault::active();

    #if ENABLE_BATTERY_MONITOR
        float v = BatteryVoltage::get_voltage();
        
        // Recover upward at BATTERY_MIN_DECAY_RATE (V/s).
        unsigned long now = millis();
        float dt = (now - last_decay_ms) / 1000.0f;
        last_decay_ms = now;
        
        min_voltage_seen += BATTERY_MIN_DECAY_RATE * dt;
        if (min_voltage_seen > v) min_voltage_seen = v;
        if (min_voltage_seen > 10.0f) min_voltage_seen = 10.0f;
        
        if (min_voltage_seen < BATTERY_CRITICAL_VOLTAGE && !estop_active) {
            estop_active = true;
            MotorFault::trigger(MotorFaultReason::BATTERY_CRITICAL);
            Comms::system.print(F("!!! BATTERY CRITICAL: "));
            Comms::system.print(min_voltage_seen);
            Comms::system.println(F("V - E-STOP !!!"));
        }
        else if (v < BATTERY_CRITICAL_VOLTAGE + 0.3f) {
            if (now - last_critical_ms >= BATTERY_CRITICAL_COOLDOWN_MS) {
                Comms::system.print(F("BATTERY NEAR CRITICAL: "));
                Comms::system.print(v);
                Comms::system.println(F("V"));
                last_critical_ms = now;
            }
        }
        else if (v < BATTERY_WARNING_VOLTAGE) {
            if (now - last_warning_ms >= BATTERY_WARNING_COOLDOWN_MS) {
                Comms::system.print(F("BATTERY LOW: "));
                Comms::system.print(v);
                Comms::system.println(F("V"));
                last_warning_ms = now;
            }
        }

        BatteryVoltage::clear_sample();
    #endif

    /* ESTOP edge: force scan state to FRONT */
    if (estop_active != last_estop) {
        if (estop_active) {
            LOG_I(Log::Ch::CH_SNR, "ESTOP: scan held (was dir %d)",
                  (int)DirectionalScan::current_scan_dir());
        }
        DirectionalScan::set_hold(estop_active);
    }
    last_estop = estop_active;
    
    SafetyState next_state;

    if (estop_active) {
        next_state = SAFETY_EMERGENCY_STOP;
    } else if (connection_loss_active) {
        next_state = SAFETY_CONNECTION_LOSS;
    } else if (input_loss_active) {
        next_state = SAFETY_INPUT_LOSS;
    } else {
        next_state = SAFETY_CLEAR;
    }

    if (next_state != current_state) {
        switch (next_state) {
            case SAFETY_EMERGENCY_STOP:
                Comms::system.println(F("!!! SAFETY: EMERGENCY STOP ACTIVE !!!"));
                break;
            case SAFETY_CONNECTION_LOSS:
                LOG_D(Log::Ch::CH_WDG, "CONNECTION_LOSS HC05_STATE");
                break;
            case SAFETY_INPUT_LOSS:
                LOG_D(Log::Ch::CH_WDG, "INPUT_LOSS WATCHDOG");
                break;
            default:
                break;
        }
    }

    current_state = next_state;
}

/* ============ STATE ACCESS ============ */
SafetyState get_state() {
    return current_state;
}

float get_min_voltage() {
    return min_voltage_seen;
}

/* ============ STATE MODIFICATION ============ */
void set_input_loss(bool active) {
    input_loss_active = active;
}

void set_connection_loss(bool active) {
    connection_loss_active = active;
}

void clear_emergency_stop() {
    emergency_stop_latched = false;
    min_voltage_seen = 10.0f;
    MotorFault::reset();
    Comms::system.println(F(">>> SAFETY: ESTOP cleared <<<"));
}

} // namespace SafetyManager
