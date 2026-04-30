/* ==================== MotorFault.cpp ==================== */
#include "control/MotorFault.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "control/MotorControl.h"
#include "safety/SafetyManager.h"
#include "comms/Comms.h"

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
        case MotorFaultReason::ESTOP:            return "E-STOP";
        case MotorFaultReason::INVALID_COMMAND:  return "INVALID COMMAND";
        case MotorFaultReason::SENSOR_FAIL:      return "SENSOR FAILED";
        case MotorFaultReason::SHIELD_NOT_FOUND: return "SHIELD NOT FOUND";
        case MotorFaultReason::INTERNAL_ERROR:   return "INTERNAL ERROR";
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
    if (fault_active) return;

    fault_active = true;
    fault_reason = reason;

    /* --- Critical Alert Output --- */
    Comms::print.println("===================");
    Comms::print.println("!!! MOTOR FAULT !!!");
    Comms::print.print(">>> "); Comms::print.print(fault_to_string(reason)); Comms::print.print(" <<<");
    Comms::print.println("===================");

    /* Immediate hardware halt */
    MotorControl::hard_stop();
}

void reset() {
    /* 
       Clear internal state. 
       SafetyManager will detect this change in its next update() 
       and allow motion again.
    */
    fault_active = false;
    fault_reason = MotorFaultReason::NONE;
    
    Comms::system.println("MotorFault RESET - System Clear");
}

} // namespace MotorFault