/* ==================== log.cpp ==================== */
#include "log/log.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/LogConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace Log {

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static uint8_t levelMask_   = 0;
static uint8_t channelMask_ = 0;

/* =============== INTERNAL HELPERS =============== */
/* ============ LOGIC ============ */
static const char lvl_char(Lvl l) {
    switch (l) {
        case Lvl::D: return 'D';
        case Lvl::I: return 'I';
        case Lvl::W: return 'W';
        case Lvl::E: return 'E';
        default:     return '?';
    }
}

/* 4 chars, space-padded */
static const char* tag_of(Ch c) {
    switch (c) {
        case Ch::CH_COM: return "COM ";
        case Ch::CH_INP: return "INP ";
        case Ch::CH_MOT: return "MOT ";
        case Ch::CH_RMP: return "RMP ";
        case Ch::CH_PID: return "PID ";
        case Ch::CH_SNR: return "SNR ";
        case Ch::CH_SAF: return "SAF ";
        case Ch::CH_WDG: return "WDG ";
        default:         return "??? ";
    }
}

/* =============== PUBLIC API =============== */
void init() {
    /* I, W, E on. D off. All channels on. */
    levelMask_   = (1u << (uint8_t)Lvl::I)
                 | (1u << (uint8_t)Lvl::W)
                 | (1u << (uint8_t)Lvl::E);
    channelMask_ = 0xFF;
}

void reset() {
    levelMask_   = 0;
    channelMask_ = 0;
}

bool enabled(Lvl l, Ch c) {
    if ((uint8_t)l >= (uint8_t)Lvl::COUNT) return false;
    if ((uint8_t)c >= (uint8_t)Ch::COUNT)  return false;
    return ((levelMask_   & (1u << (uint8_t)l)) != 0)
        && ((channelMask_ & (1u << (uint8_t)c)) != 0);
}

bool isLevelEnabled(Lvl l) {
    if ((uint8_t)l >= (uint8_t)Lvl::COUNT) {
        return false;
    }
    return (levelMask_ & (1u << (uint8_t)l)) != 0;
}

bool isChannelEnabled(Ch c) {
    if ((uint8_t)c >= (uint8_t)Ch::COUNT) {
        return false;
    }
    return (channelMask_ & (1u << (uint8_t)c)) != 0;
}

void setLevel(Lvl l, bool on) {
    if ((uint8_t)l >= (uint8_t)Lvl::COUNT) return;
    uint8_t bit = 1u << (uint8_t)l;
    if (on) levelMask_ |= bit; else levelMask_ &= ~bit;
}

void setChannel(Ch c, bool on) {
    if ((uint8_t)c >= (uint8_t)Ch::COUNT) return;
    uint8_t bit = 1u << (uint8_t)c;
    if (on) channelMask_ |= bit; else channelMask_ &= ~bit;
}

void setAllLevels(bool on)   { levelMask_   = on ? 0xFF : 0x00; }
void setAllChannels(bool on) { channelMask_ = on ? 0xFF : 0x00; }

void dump() {
    static const char* lvls[] = {"D", "I", "W", "E"};
    static const char* chs[] = {
        "COM", "INP", "MOT", "RMP", "PID", "SNR", "SAF", "WDG"
    };

    Comms::system.print(F("level mask:   0x"));
    Comms::system.println(levelMask_, HEX);
    Comms::system.print(F("channel mask: 0x"));
    Comms::system.println(channelMask_, HEX);

    for (uint8_t i = 0; i < (uint8_t)Lvl::COUNT; i++) {
        Comms::system.print(F("  "));
        Comms::system.print(lvls[i]);
        Comms::system.print(F(": "));
        Comms::system.println((levelMask_ & (1u << i)) ? F("ON") : F("OFF"));
    }
    for (uint8_t i = 0; i < (uint8_t)Ch::COUNT; i++) {
        Comms::system.print(F("  "));
        Comms::system.print(chs[i]);
        Comms::system.print(F(": "));
        Comms::system.println((channelMask_ & (1u << i)) ? F("ON") : F("OFF"));
    }
}

void dumpShort() {
    static const char* lvls[] = {"D", "I", "W", "E"};
    static const char* chs[]  = {
        "COM", "INP", "MOT", "RMP", "PID", "SNR", "SAF", "WDG"
    };

    Comms::system.print(F("Lvl:"));
    for (uint8_t i = 0; i < (uint8_t)Lvl::COUNT; i++) {
        if (levelMask_ & (1u << i)) {
            Comms::system.print(' ');
            Comms::system.print(lvls[i]);
        }
    }
    Comms::system.println();

    Comms::system.print(F("Ch: "));
    for (uint8_t i = 0; i < (uint8_t)Ch::COUNT; i++) {
        if (channelMask_ & (1u << i)) {
            Comms::system.print(chs[i]);
            Comms::system.print(' ');
        }
    }
    Comms::system.println();
}

void write(Lvl l, Ch c, const char* fmt, ...) {
    Comms::system.print('[');
    Comms::system.print(lvl_char(l));
    Comms::system.print(F("]["));
    Comms::system.print(tag_of(c));
    Comms::system.print(F("] "));

    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Comms::system.println(buf);
}

} // namespace Log
