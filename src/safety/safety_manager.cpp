/* ==================== safety_manager.cpp ==================== */
#include "config/Config.h"
#include "config/HardwareConfig.h"
#include "config/DebugConfig.h"
#include "safety/safety_manager.h"

/* =============== INCLUDES =============== */

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
static bool input_loss_active      = false;
static bool connection_loss_active = false;
static bool emergency_stop_latched = false;

/* ============ EDGE DETECTION ============ */
static bool last_input_loss_state = false;
static bool last_conn_loss_state  = false;
static bool last_estop_state      = false;

/* ============ BATTERY TRACKING ============ */
static float min_voltage_seen = 10.0f;
static unsigned long last_warning_ms = 0;
static unsigned long last_critical_ms = 0;


/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    current_state = SAFETY_CLEAR;
    emergency_stop_latched = false;
    input_loss_active = false;
    last_input_loss_state = false;
    last_estop_state = false;
    min_voltage_seen = 10.0f;
    last_warning_ms = 0;
    last_critical_ms = 0;
    Comms::system.println("SafetyManager INIT");
}

/* ============ LOGIC ============ */
void update() {
    // 1. Emergency Stop
    bool estop_active = emergency_stop_latched || MotorFault::active();

	// 2. Battery Monitoring
	#if ENABLE_BATTERY_MONITOR
		float v = BatteryVoltage::get_voltage();
		
		// Decay min_voltage_seen upward slowly (0.01V per second)
		static unsigned long last_decay_time = 0;
		unsigned long now = millis();
		float dt = (now - last_decay_time) / 1000.0f;
		last_decay_time = now;
		
		if (dt > 0.1f) {
			min_voltage_seen += BATTERY_MIN_DECAY_RATE * dt;
			if (min_voltage_seen > v) min_voltage_seen = v;
			if (min_voltage_seen > 10.0f) min_voltage_seen = 10.0f;
		}
		
		// Track downward (always)
		if (v < min_voltage_seen) min_voltage_seen = v;
		
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

    if (estop_active) {
        if (!last_estop_state) {
            Comms::system.println("!!! SAFETY: EMERGENCY STOP ACTIVE !!!");
        }
        current_state = SAFETY_EMERGENCY_STOP;
        last_estop_state = true;
        return;
    }
    last_estop_state = false;

    // 3. Connection Loss (HC-05 STATE pin)
    if (connection_loss_active) {
        #if DEBUG_WATCHDOG
            if (!last_conn_loss_state) {
                Comms::system.println("!!! SAFETY: CONNECTION LOSS (HC-05 STATE) !!!");
            }
        #endif
        current_state = SAFETY_CONNECTION_LOSS;
        last_conn_loss_state = true;
        return;
    }
    last_conn_loss_state = false;

    // 4. Input Loss (Watchdog)
    if (input_loss_active) {
        #if DEBUG_WATCHDOG
            if (!last_input_loss_state) {
                Comms::system.println("!!! SAFETY: INPUT LOSS (WATCHDOG) !!!");
            }
        #endif
        current_state = SAFETY_INPUT_LOSS;
        last_input_loss_state = true;
        return;
    }
    last_input_loss_state = false;

    // 5. All Clear
    current_state = SAFETY_CLEAR;
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