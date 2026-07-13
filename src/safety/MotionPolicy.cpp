/* ==================== MotionPolicy.cpp ==================== */
#include "safety/MotionPolicy.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/Comms.h"
#include "safety/SafetyManager.h"
#include "safety/ObstacleDetection.h"
#include "input/BluetoothSpeedAuthority.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace MotionPolicy {

/* =============== INTERNAL HELPERS =============== */
/* ============ AUTHORITY SCALING ============ */
static float compute_authority_scale() {
    float speed_scale = BluetoothSpeedAuthority::get_speed_scale();
    
    // Lean check: Block authority on any fault or signal loss
    SafetyState safety = SafetyManager::get_state();
    if (safety == SAFETY_EMERGENCY_STOP || safety == SAFETY_INPUT_LOSS) {
        return 0.0f;
    }

    // Creep bypass — if user is intentionally slow, don't scale further
    constexpr float SPEED_AUTHORITY_THRESHOLD = SPEED_AUTHORITY_THRESHOLD_USER / 1000.0f;
    if (speed_scale <= SPEED_AUTHORITY_THRESHOLD) {
        return speed_scale;
    }

    // Check if any obstacles are in slow zone
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
    
    /* ===== HARD STOPS ===== */
    SafetyState safety = SafetyManager::get_state();
    if (safety == SAFETY_EMERGENCY_STOP || safety == SAFETY_INPUT_LOSS) {
        #if DEBUG_OA_SCALE
            Comms::system.println("POLICY: HARD STOP (fault/input loss)");
        #endif
        return { 0.0f, 0.0f, 0.0f };
    }

    /* ===== OBSTACLE STOP ZONES (BACKOFF) ===== */
    Proximity front = ObstacleDetection::get_front();
    Proximity rear  = ObstacleDetection::get_rear();

    // Both sensors in stop zone — nowhere to go, hold position
    if (front.in_stop_zone && rear.in_stop_zone) {
        #if DEBUG_OA_SCALE
            Comms::system.println("POLICY: BOXED IN - HOLD");
        #endif
        return { 0.0f, 0.0f, 0.0f };
    }

    /* --- Frontal Logic --- */
    if (front.in_stop_zone) {
        if (cmd.forward > 0.0f) {
            /* User tried to go forward into obstacle */
            #if DEBUG_OA_SCALE
                Comms::system.println("POLICY: FRONT STOP - BACKOFF");
            #endif
            return { -OA_BACKOFF_SPEED, 0.0f, 0.0f };
        }
        // Reverse or strafe is fine, just block forward
        cmd.forward = min(cmd.forward, 0.0f);
    }

    /* --- Rear Logic --- */
    if (rear.in_stop_zone) {
        if (cmd.forward < 0.0f) {
            /* User tried to reverse into obstacle */
            #if DEBUG_OA_SCALE
                Comms::system.println("POLICY: REAR STOP - BACKOFF");
            #endif
            return { OA_BACKOFF_SPEED, 0.0f, 0.0f };
        }
        // Forward or strafe is fine, just block reverse
        cmd.forward = max(cmd.forward, 0.0f);
    }

    /* ===== AUTHORITY SCALING ===== */
    float scale = compute_authority_scale();
    cmd.forward *= scale;
    cmd.strafe  *= scale;
    cmd.rotate  *= scale;

    #if DEBUG_OA_SCALE
        Comms::system.print("POLICY: scale=");
        Comms::system.print(scale);
        Comms::system.print(" | cmd: F=");
        Comms::system.print(cmd.forward);
        Comms::system.print(" S=");
        Comms::system.print(cmd.strafe);
        Comms::system.print(" R=");
        Comms::system.println(cmd.rotate);
    #endif

    return cmd;
}

} // namespace MotionPolicy