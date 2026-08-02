/* ==================== DebugConfig.h ==================== */
#pragma once

/* =============== DEBUG =============== */
#define DEBUG_ENABLED  1   // Master toggle

#if DEBUG_ENABLED   // EDIT BELOW
    #define COMMS_DEBUG_MIRROR  0   // Echo commands to debug serial
    #define DEBUG_COMMS         0   // Log Bluetooth communication
    #define DEBUG_MOTOR_RAMP    0   // Print ramp calculations
    #define DEBUG_OA_REASON     0   // Print veto reasons
    #define DEBUG_OA_SCALE      0   // Print speed scaling
    #define DEBUG_SENSORS       0   // Print sensor readings
    #define DEBUG_WATCHDOG      0   // Print watchdog resets
#else   // DO NOT EDIT BELOW
    #define COMMS_DEBUG_MIRROR  0
    #define DEBUG_COMMS         0
    #define DEBUG_WATCHDOG      0
    #define DEBUG_SENSORS       0
    #define DEBUG_MOTOR_RAMP    0
    #define DEBUG_OA_REASON     0
    #define DEBUG_OA_SCALE      0
#endif
