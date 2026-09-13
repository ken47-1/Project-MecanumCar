/* ==================== encoder.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

#if ENABLE_ENCODERS

/* =============== API =============== */
namespace Encoder {

/* --------- Lifecycle --------- */
void init();
void reset();

/* --------- Direction --------- */
/* Called by MotorControl before every update. -1 reverse, 0 stop, +1 forward. */
void set_direction(uint8_t idx, int8_t dir);

/* --------- Read --------- */
int32_t get_count(uint8_t idx);
float   get_rpm(uint8_t idx);   /* Signed. Follows the last commanded direction. */

}

#else

/* =============== STUBS =============== */
namespace Encoder {
    inline void init() {}
    inline void reset() {}
    inline void set_direction(uint8_t, int8_t) {}
    inline int32_t get_count(uint8_t) { return 0; }
    inline float   get_rpm(uint8_t)   { return 0.0f; }
}

#endif // ENABLE_ENCODERS