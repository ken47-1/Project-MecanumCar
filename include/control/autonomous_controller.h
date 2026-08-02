/* ==================== autonomous_controller.h ==================== */
#pragma once

#if ENABLE_AUTONOMOUS_MODE

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "input/input_watchdog.h"

/* =============== API =============== */
namespace AutonomousController {
    /* --------- Lifecycle --------- */
    void reset();
    
    /* --------- Logic --------- */
    // Note: Takes watchdog reference to handle automatic keep-alive
    void update(InputWatchdog& watchdog);
}

#endif // ENABLE_AUTONOMOUS_MODE