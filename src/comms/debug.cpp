/* ==================== debug.cpp ==================== */
#include "comms/debug.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/DebugConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace Debug {

/* =============== INTERNAL HELPERS =============== */
/* ============ TAGS ============ */
static const char* tag_of(Ch c) {
    switch (c) {
        case Ch::COMMS:    return "COM";
        case Ch::IN:       return "INP";
        case Ch::MOTOR:    return "MOT";
        case Ch::RAMP:     return "RMP";
        case Ch::PID:      return "PID";
        case Ch::SENSORS:  return "SNR";
        case Ch::SAFETY:   return "SAF";
        case Ch::WATCHDOG: return "WDG";
        default:           return "???";
    }
}

/* =============== INTERNAL STATE =============== */
static uint8_t mask_ = 0;

/* =============== PUBLIC API =============== */
void init() {
    mask_ = 0;
}

void reset() {
    mask_ = 0;
}

bool enabled(Ch c) {
    if ((uint8_t)c >= (uint8_t)Ch::COUNT) {
        return false;
    }
    return (mask_ & (1u << (uint8_t)c)) != 0;
}

void set(Ch c, bool on) {
    if ((uint8_t)c >= (uint8_t)Ch::COUNT) {
        return;
    }
    uint8_t bit = 1u << (uint8_t)c;
    if (on) {
        mask_ |= bit;
    } else {
        mask_ &= ~bit;
    }
}

void toggle(Ch c) {
    set(c, !enabled(c));
}

void set_all(bool on) {
    mask_ = on ? 0xFF : 0x00;
}

void dump_short() {
    Comms::system.print(F("Debug mask: 0x"));
    Comms::system.println(mask_, HEX);
}

void dump() {
    dump_short();

    const char* names[] = {
        "COMMS", "INPUT", "MOTOR", "RAMP",
        "PID", "SENSORS", "SAFETY", "WATCHDOG"
    };

    for (uint8_t i = 0; i < (uint8_t)Ch::COUNT; i++) {
        Comms::system.print(F("  "));
        Comms::system.print(names[i]);
        Comms::system.print(F(": "));
        Comms::system.println(enabled((Ch)i) ? F("ON") : F("OFF"));
    }
}

void emit(Ch c, const char* fmt, ...) {
    Comms::system.print('[');
    Comms::system.print(tag_of(c));
    Comms::system.print(F("] "));

    char buf[48];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Comms::system.println(buf);
}

} // namespace Debug