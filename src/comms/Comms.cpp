/* ==================== Comms.cpp ==================== */
#include "comms/Comms.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/MultiPrint.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
#ifdef BOARD_UNO_R4
  static HardwareSerial& bt_serial = Serial1;
#else
  static HardwareSerial& bt_serial = Serial;
#endif

static MultiPrint comms_out(&bt_serial);
static MultiPrint system_out_impl(&bt_serial, &Serial);

/* =============== INTERNAL HELPERS =============== */
#if COMMS_DEBUG_MIRROR
static bool usb_serial_enabled = false;

static void ensure_usb_serial() {
    if (!usb_serial_enabled) {
        Serial.begin(9600);
        usb_serial_enabled = true;
    }
}
#endif

/* ===== PUBLIC CHANNELS ===== */
namespace Comms {
Print& print  = comms_out;
Print& system = system_out_impl;
}

/* =============== PUBLIC API =============== */
namespace Comms {

void begin() {
    bt_serial.begin(9600);

#if COMMS_DEBUG_MIRROR
    ensure_usb_serial();
    comms_out.set_secondary(&Serial);
#endif

#if ENABLE_HC05_STATE_PIN
    pinMode(BT_STATE_PIN, INPUT);
#endif
}

bool available() {
    return bt_serial.available();
}

int read() {
    return bt_serial.read();
}

#if ENABLE_HC05_STATE_PIN
bool is_connected() {
    return digitalRead(BT_STATE_PIN) == HIGH;
}
#else
bool is_connected() {
    return true;  // HC-06: assume always connected
}
#endif

} // namespace Comms