/* ==================== motor_fault.cpp ==================== */
#include "control/motor_fault.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */
#include "control/motor_control.h"
#include "safety/safety_manager.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace MotorFault {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static bool             fault_active = false;
static MotorFaultReason fault_reason = MotorFaultReason::NONE;

/* =============== INTERNAL HELPERS =============== */
/* ============ STRINGS ============ */
static const char* fault_to_string(MotorFaultReason r) {
    switch (r) {

        // Hardware failures (fatal)
        case MotorFaultReason::SHIELD_NOT_FOUND: return "SHIELD NOT FOUND";
        case MotorFaultReason::INTERNAL_ERROR:   return "INTERNAL ERROR";
        case MotorFaultReason::BATTERY_CRITICAL: return "BATTERY CRITICAL";
        case MotorFaultReason::SENSOR_FAIL:      return "SENSOR FAILED";

        // User/command issues
        case MotorFaultReason::ESTOP:            return "E-STOP";
        case MotorFaultReason::INVALID_COMMAND:  return "INVALID COMMAND";
        case MotorFaultReason::MANUAL:           return "MANUAL";
        default:                                 return "UNKNOWN";
    }
}

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    fault_active = false;
    fault_reason = MotorFaultReason::NONE;
    Comms::system.println("MotorFault INIT");
}

/* ============ STATUS ============ */
bool active() {
    return fault_active;
}

MotorFaultReason reason() {
    return fault_reason;
}

/* ============ CONTROL ============ */
void trigger(MotorFaultReason reason) {
    /* Prevent re-triggering if already faulted */
    if (fault_active) {
        Comms::system.print(">>> Fault ignored (already active): ");
        Comms::system.println(fault_to_string(reason));
        return;
    }

    fault_active = true;
    fault_reason = reason;

    /* --- Critical Alert Output --- */
    Comms::print.println("===================");
    Comms::print.println("!!! MOTOR FAULT !!!");
    Comms::print.print(">>> ");
    Comms::print.print(fault_to_string(reason));
    Comms::print.println(" <<<");
    Comms::print.println("===================");

    /* Immediate hardware halt */
    MotorControl::hard_stop();
}

void reset() {
    // SafetyManager reads this on its next update and allows motion again.
    fault_active = false;
    fault_reason = MotorFaultReason::NONE;
    
    Comms::system.println("MotorFault RESET - System Clear");
}

} // namespace MotorFault
