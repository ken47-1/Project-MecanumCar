/* ==================== LogConfig.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CORE ============ */
#include <stdint.h>

/* =============== FEATURES =============== */
/* Compile-time gates. Set to 0 to strip logging entirely from the build. */
#define LOG_ENABLED        0   // Master switch for all LOG_* macros
#define COMMS_DEBUG_MIRROR 1   // Mirror Bluetooth output to USB Serial (R4)

/* =============== LIMITS =============== */

/* Maximum length of a single formatted log line (bytes). 
   Lines longer than this are truncated. */
constexpr uint8_t LOG_BUFFER_SIZE = 64;

/* Rate limits (ms). Prevents serial flooding during high-frequency loops. 
   0 = unlimited. Math: 1000 / LIMIT = max messages per second. */
constexpr unsigned long LOG_RATE_LIMIT_D = 100;  // 10/s max (Debug)
constexpr unsigned long LOG_RATE_LIMIT_I = 200;  // 5/s max  (Info)
constexpr unsigned long LOG_RATE_LIMIT_W = 0;    // Unlimited (Warn)
constexpr unsigned long LOG_RATE_LIMIT_E = 0;    // Unlimited (Error)