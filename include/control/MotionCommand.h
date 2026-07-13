/* ==================== MotionCommand.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <stdint.h>

/* =============== API =============== */
struct MotionCommand {
    float forward;   // -1 Backward, +1 Forward
    float strafe;    // -1 Left, +1 Right
    float rotate;    // -1 CCW, +1 CW
};
