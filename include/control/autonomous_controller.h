/* ==================== autonomous_controller.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* =============== TYPES =============== */
/* ============ FORWARD DECLS ============ */
class InputWatchdog;

#if ENABLE_AUTONOMOUS_MODE

/* =============== API =============== */
namespace AutonomousController {
    /* ============ Lifecycle ============ */
    void reset();

    /* ============ Logic ============ */
    void update(InputWatchdog& watchdog);
}

#else

/* =============== API =============== */
namespace AutonomousController {
    inline void reset() {}
    inline void update(InputWatchdog&) {}
}

#endif // ENABLE_AUTONOMOUS_MODE
