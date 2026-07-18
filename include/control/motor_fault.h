/* ==================== motor_fault.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum class MotorFaultReason {
    NONE,
    ESTOP,
    INVALID_COMMAND,
    SENSOR_FAIL,
    SHIELD_NOT_FOUND,
    INTERNAL_ERROR,
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