/* ==================== system_fault.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum class SystemFaultReason {
    NONE,

    // Hardware failures (fatal)
    SHIELD_NOT_FOUND,
    INTERNAL_ERROR,
    BATTERY_CRITICAL,
    SENSOR_FAIL,
    
    // User/command issues
    ESTOP,
    INVALID_COMMAND,
    MANUAL
};

/* =============== API =============== */
namespace SystemFault {
    /* ============ Lifecycle ============ */
    void init();

    /* ============ Status ============ */
    bool active();
    SystemFaultReason reason();

    /* ============ Control ============ */
    void trigger(SystemFaultReason reason);
    void reset();
    void reset_user_faults();
}
