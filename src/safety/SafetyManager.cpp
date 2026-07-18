/* ==================== SafetyManager.cpp ==================== */
#include "safety/SafetyManager.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/Comms.h"
#include "control/MotorFault.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace SafetyManager {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static SafetyState current_state          = SAFETY_CLEAR;
static bool        input_loss_active      = false;
static bool        connection_loss_active = false;
static bool        emergency_stop_latched = false;

/* ============ EDGE DETECTION ============ */
static bool        last_input_loss_state = false;
static bool        last_conn_loss_state  = false;
static bool        last_estop_state      = false;

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    current_state = SAFETY_CLEAR;
    emergency_stop_latched = false;
    input_loss_active = false;
    last_input_loss_state = false;
    last_estop_state = false;
    Comms::system.println("SafetyManager INIT");
}

/* ============ LOGIC ============ */
void update() {
    // 1. Emergency Stop
    bool estop_active = emergency_stop_latched || MotorFault::active();
    if (estop_active) {
        if (!last_estop_state) {
            Comms::system.println("!!! SAFETY: EMERGENCY STOP ACTIVE !!!");
        }
        current_state = SAFETY_EMERGENCY_STOP;
        last_estop_state = true;
        return;
    }
    last_estop_state = false;

    // 2. Connection Loss (HC-05 STATE pin)
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

    // 3. Input Loss (Watchdog)
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

    // 4. All Clear
    current_state = SAFETY_CLEAR;
}

/* ============ STATE ACCESS ============ */
SafetyState get_state() {
    return current_state;
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
    MotorFault::reset();
    Comms::system.println(">>> SAFETY: ESTOP cleared <<<");
}

} // namespace SafetyManager