/* ==================== safety_manager.cpp ==================== */
#include "safety/safety_manager.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"
#include "config/HardwareConfig.h"
#include "config/DebugConfig.h"

/* ============ PROJECT ============ */
#include "comms/comms.h"
#include "control/motor_fault.h"
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
    Comms::system.println("SafetyManager INIT");
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
        
        // Critical: E-stop if min voltage drops below threshold
        if (min_voltage_seen < BATTERY_CRITICAL_VOLTAGE && !estop_active) {
            estop_active = true;
            MotorFault::trigger(MotorFaultReason::BATTERY_CRITICAL);
            Comms::system.print("!!! BATTERY CRITICAL: ");
            Comms::system.print(min_voltage_seen);
            Comms::system.println("V - E-STOP !!!");
        }
        // Critical warning (pre-E-stop)
        else if (v < BATTERY_CRITICAL_VOLTAGE + 0.3f) {
            if (now - last_critical_ms >= BATTERY_CRITICAL_COOLDOWN_MS) {
                Comms::system.print("BATTERY NEAR CRITICAL: ");
                Comms::system.print(v);
                Comms::system.println("V");
                last_critical_ms = now;
            }
        }
        // Warning only
        else if (v < BATTERY_WARNING_VOLTAGE) {
            if (now - last_warning_ms >= BATTERY_WARNING_COOLDOWN_MS) {
                Comms::system.print("BATTERY LOW: ");
                Comms::system.print(v);
                Comms::system.println("V");
                last_warning_ms = now;
            }
        }
    #endif

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
        #if DEBUG_WATCHDOG
            switch (next_state) {
                case SAFETY_EMERGENCY_STOP:
                    Comms::system.println("!!! SAFETY: EMERGENCY STOP ACTIVE !!!");
                    break;
                case SAFETY_CONNECTION_LOSS:
                    Comms::system.println("!!! SAFETY: CONNECTION LOSS (HC-05 STATE) !!!");
                    break;
                case SAFETY_INPUT_LOSS:
                    Comms::system.println("!!! SAFETY: INPUT LOSS (WATCHDOG) !!!");
                    break;
                default:
                    break;
            }
        #else
            if (next_state == SAFETY_EMERGENCY_STOP) {
                Comms::system.println("!!! SAFETY: EMERGENCY STOP ACTIVE !!!");
            }
        #endif
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

void reset_min_voltage() {
    min_voltage_seen = 10.0f;
}

/* ============ STATE MODIFICATION ============ */
void set_input_loss(bool active) {
    input_loss_active = active;
}

void set_connection_loss(bool active) {
    connection_loss_active = active;
}

void set_emergency_stop() {
    emergency_stop_latched = true;
    MotorFault::trigger(MotorFaultReason::ESTOP);
}

void clear_emergency_stop() {
    emergency_stop_latched = false;
    min_voltage_seen = 10.0f;  // Reset on E-stop clear
    MotorFault::reset();
    Comms::system.println(">>> SAFETY: ESTOP cleared <<<");
}

} // namespace SafetyManager
