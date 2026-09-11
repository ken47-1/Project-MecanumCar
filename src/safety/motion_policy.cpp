/* ==================== motion_policy.cpp ==================== */
#include "safety/motion_policy.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */
#include "comms/comms.h"
#include "safety/safety_manager.h"
#include "safety/obstacle_detection.h"
#include "input/bluetooth_speed_authority.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace MotionPolicy {

/* =============== INTERNAL HELPERS =============== */
/* ============ AUTHORITY SCALING ============ */
static float compute_authority_scale() {
    float speed_scale = BluetoothSpeedAuthority::get_speed_scale();
    
    SafetyState safety = SafetyManager::get_state();
    if (safety == SAFETY_EMERGENCY_STOP ||
        safety == SAFETY_INPUT_LOSS ||
        safety == SAFETY_CONNECTION_LOSS) {
        return 0.0f;
    }

    constexpr float SPEED_AUTHORITY_THRESHOLD = SPEED_AUTHORITY_THRESHOLD_USER / 1000.0f;
    if (speed_scale <= SPEED_AUTHORITY_THRESHOLD) {
        return speed_scale;
    }

    Proximity front = ObstacleDetection::get_front();
    Proximity rear  = ObstacleDetection::get_rear();
        
    bool any_slow = front.in_slow_zone || rear.in_slow_zone;
        
    if (any_slow) {
        speed_scale *= OA_SOFT_AUTHORITY;
    }

    return constrain(speed_scale, 0.0f, 1.0f);
}

/* =============== PUBLIC API =============== */
MotionCommand apply_safety(MotionCommand cmd) {
    
    /* --- HARD STOPS --- */
    SafetyState safety = SafetyManager::get_state();
    if (safety == SAFETY_EMERGENCY_STOP ||
        safety == SAFETY_INPUT_LOSS ||
        safety == SAFETY_CONNECTION_LOSS) {
        #if DEBUG_OA_SCALE
            Comms::system.println("POLICY: HARD STOP (fault/input loss/conn loss)");
        #endif
        return { 0.0f, 0.0f, 0.0f };
    }

    /* --- OBSTACLE STOP ZONES (STATELESS) --- */
    #if ENABLE_OBSTACLE_AVOIDANCE
        Proximity front = ObstacleDetection::get_front();
        Proximity rear  = ObstacleDetection::get_rear();

        // Both sensors in stop zone → stop
        if (front.in_stop_zone && rear.in_stop_zone) {
            return { 0.0f, 0.0f, 0.0f };
        }

        // Front stop zone
        if (front.in_stop_zone) {
            if (cmd.forward > 0.0f) {
                return { 0.0f, 0.0f, 0.0f };
            }
            // Block forward but allow reverse/strafe
            cmd.forward = min(cmd.forward, 0.0f);
        }

        // Rear stop zone
        #if ENABLE_ULTRASONIC_REAR
            if (rear.in_stop_zone) {
                if (cmd.forward < 0.0f) {
                    return { 0.0f, 0.0f, 0.0f };
                }
                cmd.forward = max(cmd.forward, 0.0f);
            }
        #endif // ENABLE_ULTRASONIC_REAR
    #endif // ENABLE_OBSTACLE_AVOIDANCE

    /* --- AUTHORITY SCALING --- */
    float scale = compute_authority_scale();
    cmd.forward *= scale;
    cmd.strafe  *= scale;
    cmd.rotate  *= scale;

    #if DEBUG_OA_SCALE
        char buf[80];
        snprintf(buf, sizeof(buf), "POLICY: scale=%.2f | F=%.2f S=%.2f R=%.2f",
                scale, cmd.forward, cmd.strafe, cmd.rotate);
        Comms::system.println(buf);
    #endif

    return cmd;
}

} // namespace MotionPolicy
