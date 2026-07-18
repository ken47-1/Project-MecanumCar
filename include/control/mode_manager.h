/* ==================== mode_manager.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ CORE ============ */
#include <stdint.h>

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum class DriveMode : uint8_t {
    MANUAL,
    AUTONOMOUS
};

/* =============== API =============== */
namespace ModeManager {
    /* ========= LIFECYCLE ========= */
    void init();

    /* ========= STATE ========= */
    void set(DriveMode mode);
    DriveMode get();
    bool is_autonomous();
}
