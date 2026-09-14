/* ==================== log.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/LogConfig.h"

/* ============ CORE ============ */
#include <stdint.h>

/* =============== API =============== */
namespace Log {

/* =============== TYPES =============== */
/* ============ ENUMS ============ */
enum class Lvl : uint8_t { D = 0, I = 1, W = 2, E = 3, COUNT = 4 };

enum class Ch : uint8_t {
    CH_COM = 0,   /* Comms */
    CH_INP = 1,   /* Input, buttons, speed, watchdog */
    CH_MOT = 2,   /* MotorControl, MotorHardware */
    CH_RMP = 3,   /* MotorRamp */
    CH_PID = 4,   /* MotorPID, Encoder */
    CH_SNR = 5,   /* Ultrasonic, ObstacleDetection, Battery */
    CH_SAF = 6,   /* SafetyManager, MotionPolicy, MotorFault */
    CH_WDG = 7,   /* InputWatchdog */
    COUNT  = 8
};

/* =============== API =============== */
void init();
void reset();

bool enabled(Lvl l, Ch c);
bool isLevelEnabled(Lvl l);
bool isChannelEnabled(Ch c);

void setLevel(Lvl l, bool on);
void setChannel(Ch c, bool on);
void setAllLevels(bool on);
void setAllChannels(bool on);

void dump();
void dumpShort();
void write(Lvl l, Ch c, const char* fmt, ...);

} // namespace Log

/* =============== MACROS =============== */
#if LOG_ENABLED

#define LOG_D(ch, ...) do { if (Log::enabled(Log::Lvl::D, ch)) Log::write(Log::Lvl::D, ch, __VA_ARGS__); } while (0)
#define LOG_I(ch, ...) do { if (Log::enabled(Log::Lvl::I, ch)) Log::write(Log::Lvl::I, ch, __VA_ARGS__); } while (0)
#define LOG_W(ch, ...) do { if (Log::enabled(Log::Lvl::W, ch)) Log::write(Log::Lvl::W, ch, __VA_ARGS__); } while (0)
#define LOG_E(ch, ...) do { if (Log::enabled(Log::Lvl::E, ch)) Log::write(Log::Lvl::E, ch, __VA_ARGS__); } while (0)

#else

#define LOG_D(ch, ...) do {} while (0)
#define LOG_I(ch, ...) do {} while (0)
#define LOG_W(ch, ...) do {} while (0)
#define LOG_E(ch, ...) do {} while (0)

#endif // LOG_ENABLED
