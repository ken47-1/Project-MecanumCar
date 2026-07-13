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
static SafetyState current_state           = SAFETY_CLEAR;
static bool        emergency_stop_latched  = false;
static bool        input_loss_active       = false;

/* ============ EDGE DETECTION ============ */
static bool        last_input_loss_state   = false;
static bool        last_estop_state        = false;

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
    // 1. Check for Hardware/ESTOP Faults
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

    // 2. Check for Bluetooth Input Loss
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
    
    #if DEBUG_WATCHDOG
        if (last_input_loss_state) {
            Comms::system.println(">>> SAFETY: SIGNAL RESTORED <<<");
        }
    #endif
    last_input_loss_state = false;

    // 3. All Clear
    current_state = SAFETY_CLEAR;
}

/* ============ STATE ACCESS ============ */
SafetyState get_state() {
    return current_state;
}

/* ============ STATE MODIFICATION ============ */
void set_emergency_stop() {
    emergency_stop_latched = true;
    MotorFault::trigger(MotorFaultReason::ESTOP);
}

void clear_emergency_stop() {
    emergency_stop_latched = false;
    MotorFault::reset();
    Comms::system.println(">>> SAFETY: ESTOP cleared <<<");
}

void set_input_loss(bool active) {
    input_loss_active = active;
}

} // namespace SafetyManager