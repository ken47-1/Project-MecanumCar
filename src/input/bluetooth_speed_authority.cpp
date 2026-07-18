/* ==================== bluetooth_speed_authority.cpp ==================== */
#include "input/bluetooth_speed_authority.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/comms.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
// User-facing speed (0–1000)
static uint16_t speed_user =
    constrain(SPEED_USER_DEFAULT, SPEED_USER_MIN, SPEED_USER_MAX);

// Step size in user units
static uint16_t speed_step = SPEED_STEP_NORMAL;

static bool awaiting_speed_cmd = false;

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

namespace BluetoothSpeedAuthority {

bool handle_char(char c) {
#if !ENABLE_INPUT_SPEED_AUTHORITY
    return false;
#endif

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
    }

    return true;
}

/* =============== PUBLIC API =============== */
// Normalized authority [0.0 – 1.0]
float get_speed_scale() {
    return (float)speed_user / (float)SPEED_USER_MAX;
}

} // namespace BluetoothSpeedAuthority