/* ==================== MotionPolicy.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ==================== PROJECT ==================== */
#include "control/MotionCommand.h"

/* =============== API =============== */
namespace MotionPolicy {
    /* ========= Safety Enforcement ========= */
    // Applies all safety rules: emergency stop, input loss, obstacle zones
    // Returns the command that should actually be executed
    MotionCommand apply_safety(MotionCommand cmd);
}
