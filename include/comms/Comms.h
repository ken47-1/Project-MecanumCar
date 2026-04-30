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

    /* --------- Output Channels --------- */
    extern Print& print;
    extern Print& system;
}