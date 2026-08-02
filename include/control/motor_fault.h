/* ==================== motor_fault.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum class MotorFaultReason {
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
namespace MotorFault {
    /* --------- Lifecycle --------- */
    void init();

    /* --------- Status --------- */
    bool active();
    MotorFaultReason reason();

    /* --------- Control --------- */
    void trigger(MotorFaultReason reason);
    void reset();
}