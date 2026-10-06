/* ==================== LogConfig.h ==================== */
#pragma once

/* =============== LOG =============== */
#define LOG_ENABLED        1   // Compile-time gate for LOG_* macros
#define COMMS_DEBUG_MIRROR 0   // Mirror serial output to USB (R4 only)

/* Rate limits: minimum ms between emits, per level. 0 = no limit. */
#define LOG_RATE_LIMIT_D   100  // Debug: 10/sec max
#define LOG_RATE_LIMIT_I   200  // Info:  5/sec max
#define LOG_RATE_LIMIT_W   0    // Warn:  unlimited
#define LOG_RATE_LIMIT_E   0    // Error: unlimited
