/* ==================== motor_fault.cpp ==================== */
#include "control/motor_fault.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/motor_control.h"

/* ========= SAFETY ========= */
#include "safety/safety_manager.h"

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
    Comms::system.println(F("MotorFault INIT"));
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
        Comms::system.print(F(">>> Fault ignored (already active): "));
        Comms::system.println(fault_to_string(reason));
        return;
    }

    fault_active = true;
    fault_reason = reason;

    /* --- Critical Alert Output --- */
    Comms::print.println(F("==================="));
    Comms::print.println(F("!!! MOTOR FAULT !!!"));
    Comms::print.print(F(">>> "));
    Comms::print.print(fault_to_string(reason));
    Comms::print.println(F(" <<<"));
    Comms::print.println(F("==================="));

    /* Immediate hardware halt */
    MotorControl::hard_stop();
}

void reset() {
    fault_active = false;
    fault_reason = MotorFaultReason::NONE;

    Comms::system.println(F("MotorFault RESET - System Clear"));
}

} // namespace MotorFault
