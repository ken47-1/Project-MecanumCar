/* ==================== SafetyManager.h ==================== */
#pragma once

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum SafetyState {
    SAFETY_CLEAR,
    SAFETY_INPUT_LOSS,
    SAFETY_EMERGENCY_STOP
};

/* =============== API =============== */
namespace SafetyManager {
    /* --------- Lifecycle --------- */
    void init();
    
    /* --------- Logic / Polling --------- */
    void update();
    
    /* --------- State Access --------- */
    SafetyState get_state();

    /* --------- State Modification --------- */
    void set_emergency_stop();
    void clear_emergency_stop();
    void set_input_loss(bool active);
}