/* ==================== input_watchdog.h ==================== */
#pragma once

#include "config/Config.h"

#if ENABLE_INPUT_WATCHDOG

/* =============== INCLUDES =============== */

/* ============ CORE ============ */
#include <stdint.h>

/* =============== API =============== */

/* ============ CLASSES ============ */
class InputWatchdog {
public:
    explicit InputWatchdog(uint32_t timeout_ms);

    /* --------- Control --------- */
    void feed();
    void reset();
    void enable(bool state);
    void set_motion_state(bool active);
    void update();

    /* --------- Status --------- */
    bool is_expired() const;

private:
    uint32_t _timeout_ms;
    uint32_t _last_seen;
    bool     _armed;
    bool     _enabled;
    bool     _motion_active;
    uint32_t _now() const;
};

#else

/* ============ STUBS ============ */
class InputWatchdog {
public:
    explicit InputWatchdog(uint32_t) {}
    void feed() {}
    void reset() {}
    void enable(bool) {}
    void update() {}
    bool is_expired() const { return false; }
};

#endif