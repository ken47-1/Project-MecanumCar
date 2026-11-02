/* ==================== odometry.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* =============== TYPES =============== */
struct Pose {
    float x_mm;
    float y_mm;
    float heading_rad;
};

#if ENABLE_ENCODERS

/* =============== API =============== */
namespace Odometry {
    void init();
    void reset();
    void update();
    Pose get_pose();
}

#else

/* =============== STUBS =============== */
namespace Odometry {
    inline void init() {}
    inline void reset() {}
    inline void update() {}
    inline Pose get_pose() { return {0, 0, 0}; }
}

#endif // ENABLE_ENCODERS