/* ==================== odometry.cpp ==================== */
#include "sensors/odometry.h"

#if ENABLE_ENCODERS

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= SENSORS ========= */
#include "sensors/encoder.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace Odometry {

/* =============== INTERNAL STATE =============== */
static Pose current_pose = {0.0f, 0.0f, 0.0f};
static int32_t last_counts[4] = {0, 0, 0, 0};

/* =============== INTERNAL HELPERS =============== */
static float ticks_to_mm(int32_t ticks) {
    constexpr float WHEEL_CIRC_MM = PI * WHEEL_DIAMETER_MM;
    return (float)ticks * (WHEEL_CIRC_MM / (float)ENCODER_TICKS_PER_REV);
}

/* =============== PUBLIC API =============== */
void init() {
    reset();
}

void reset() {
    current_pose = {0.0f, 0.0f, 0.0f};
    for(uint8_t i=0; i<4; i++) {
        last_counts[i] = Encoder::get_count(i);
    }
}

void update() {
    int32_t cur_counts[4];
    float d_mm[4];

    for(uint8_t i=0; i<4; i++) {
        cur_counts[i] = Encoder::get_count(i);
        d_mm[i] = ticks_to_mm(cur_counts[i] - last_counts[i]);
        last_counts[i] = cur_counts[i];
    }

    /* Mecanum forward kinematics */
    float d_fl = d_mm[0];
    float d_fr = d_mm[1];
    float d_rl = d_mm[2];
    float d_rr = d_mm[3];

    float d_forward = ( d_fl + d_fr + d_rl + d_rr) / 4.0f;
    float d_strafe  = (-d_fl + d_fr + d_rl - d_rr) / 4.0f;
    float d_theta   = (-d_fl + d_fr - d_rl + d_rr) / (4.0f * (TRACK_WIDTH_MM / 2.0f));

    /* Integrate pose (midpoint approximation) */
    float mid_theta = current_pose.heading_rad + (d_theta / 2.0f);

    current_pose.x_mm += (d_forward * cos(mid_theta)) - (d_strafe * sin(mid_theta));
    current_pose.y_mm += (d_forward * sin(mid_theta)) + (d_strafe * cos(mid_theta));
    current_pose.heading_rad += d_theta;
}

Pose get_pose() {
    return current_pose;
}

} // namespace Odometry

#endif // ENABLE_ENCODERS