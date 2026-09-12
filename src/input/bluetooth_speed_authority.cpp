/* ==================== bluetooth_speed_authority.cpp ==================== */
#include "input/bluetooth_speed_authority.h"

#if ENABLE_INPUT_SPEED_AUTHORITY

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

namespace BluetoothSpeedAuthority {

/* =============== INTERNAL STATE =============== */
static uint16_t speed_user = constrain(SPEED_USER_DEFAULT, SPEED_USER_MIN, SPEED_USER_MAX);
static uint16_t speed_step = SPEED_STEP_NORMAL;
static bool awaiting_speed_cmd = false;
static unsigned long last_feedback_ms = 0;

/* =============== INTERNAL HELPERS =============== */
static void send_speed_feedback() {
    Comms::print.print("*G");
    Comms::print.print(speed_user);
    Comms::print.println("*");

    Comms::print.print("*%");
    if (speed_step == SPEED_STEP_FINE) {
        Comms::print.print("Fine");
    } else if (speed_step == SPEED_STEP_NORMAL) {
        Comms::print.print("Normal");
    } else {
        Comms::print.print("Rough");
    }
    Comms::print.println("*");
}

/* =============== PUBLIC API =============== */
bool handle_char(char c) {
    if (c == '%') {
        awaiting_speed_cmd = true;
        return true;
    }

    if (!awaiting_speed_cmd) return false;
    awaiting_speed_cmd = false;

    uint16_t old_speed = speed_user;
    uint16_t old_step  = speed_step;

    switch (c) {
        case '+': {
            int32_t v = speed_user + speed_step;
            speed_user = constrain(v, SPEED_USER_MIN, SPEED_USER_MAX);
            break;
        }

        case '-': {
            int32_t v = speed_user - speed_step;
            speed_user = constrain(v, SPEED_USER_MIN, SPEED_USER_MAX);
            break;
        }

        case 'R': speed_step = SPEED_STEP_ROUGH;  break;
        case 'N': speed_step = SPEED_STEP_NORMAL; break;
        case 'F': speed_step = SPEED_STEP_FINE;   break;

        default:
            return false;
    }

    if (speed_user != old_speed || speed_step != old_step) {
        send_speed_feedback();
        last_feedback_ms = millis();
    }

    return true;
}

// Normalized authority [0.0 – 1.0]
float get_speed_scale() {
    return (float)speed_user / (float)SPEED_USER_MAX;
}

void feedback_tick() {
    unsigned long now = millis();
    if (now - last_feedback_ms < SPEED_FEEDBACK_INTERVAL_MS) {
        return;
    }
    last_feedback_ms = now;
    send_speed_feedback();
}

} // namespace BluetoothSpeedAuthority

#endif // ENABLE_INPUT_SPEED_AUTHORITY
