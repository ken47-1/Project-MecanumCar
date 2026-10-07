/* ==================== system_fault.cpp ==================== */
#include "safety/system_fault.h"

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/motor_control.h"

/* ========= SAFETY ========= */
#include "safety/safety_manager.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace SystemFault {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static bool             fault_active = false;
static SystemFaultReason fault_reason = SystemFaultReason::NONE;

/* =============== INTERNAL HELPERS =============== */
/* ============ STRINGS ============ */
static const char* fault_to_string(SystemFaultReason r) {
    switch (r) {

        // Hardware failures (fatal)
        case SystemFaultReason::SHIELD_NOT_FOUND: return "SHIELD NOT FOUND";
        case SystemFaultReason::INTERNAL_ERROR:   return "INTERNAL ERROR";
        case SystemFaultReason::BATTERY_CRITICAL: return "BATTERY CRITICAL";
        case SystemFaultReason::SENSOR_FAIL:      return "SENSOR FAILED";

        // User/command issues
        case SystemFaultReason::ESTOP:            return "E-STOP";
        case SystemFaultReason::INVALID_COMMAND:  return "INVALID COMMAND";
        case SystemFaultReason::MANUAL:           return "MANUAL";
        default:                                 return "UNKNOWN";
    }
}

/* =============== PUBLIC API =============== */
/* ============ LIFECYCLE ============ */
void init() {
    fault_active = false;
    fault_reason = SystemFaultReason::NONE;
    Comms::system.println(F("SystemFault INIT"));
}

/* ============ STATUS ============ */
bool active() {
    return fault_active;
}

SystemFaultReason reason() {
    return fault_reason;
}

/* ============ CONTROL ============ */
void trigger(SystemFaultReason reason) {
    /* Prevent re-triggering if already faulted */
    if (fault_active) {
        Comms::system.print(F(">>> Fault ignored (already active): "));
        Comms::system.println(fault_to_string(reason));
        return;
    }

    fault_active = true;
    fault_reason = reason;

    /* --- Critical Alert Output --- */
    Comms::print.println(F("===================="));
    Comms::print.println(F("!!! SYSTEM FAULT !!!"));
    Comms::print.println(fault_to_string(reason));
    Comms::print.println(F("===================="));

    /* Immediate hardware halt */
    MotorControl::hard_stop();
    SafetyManager::refresh();
}

void reset() {
    fault_active = false;
    fault_reason = SystemFaultReason::NONE;

    Comms::system.println(F("SystemFault CLEAR - all faults"));
    SafetyManager::refresh();
}

void reset_user_faults() {
    /* Hardware faults stay latched. Only user faults clear. */
    switch (fault_reason) {
        case SystemFaultReason::SHIELD_NOT_FOUND:
        case SystemFaultReason::INTERNAL_ERROR:
        case SystemFaultReason::BATTERY_CRITICAL:
        case SystemFaultReason::SENSOR_FAIL:
            Comms::system.print(F("Cannot clear hardware fault: "));
            Comms::system.println(fault_to_string(fault_reason));
            break;
        case SystemFaultReason::NONE:
            Comms::system.println(F("SystemFault CLEAR - no fault"));
            break;
        default:
            fault_active = false;
            fault_reason = SystemFaultReason::NONE;
            Comms::system.println(F("SystemFault CLEAR - user fault"));
            break;
    }

    SafetyManager::refresh();
}

} // namespace SystemFault
