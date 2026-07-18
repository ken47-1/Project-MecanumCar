/* ==================== Comms.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ CORE ============ */
#include <Arduino.h>

/* =============== API =============== */
namespace Comms {
    /* --------- Lifecycle --------- */
    void begin();

    /* --------- Input --------- */
    bool available();
    int  read();

    /* --------- Status --------- */
    bool is_connected();  // Returns true if HC-05 is connected (STATE pin HIGH)

    /* --------- Output Channels --------- */
    extern Print& print;
    extern Print& system;
}