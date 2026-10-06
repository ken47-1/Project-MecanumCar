/* ==================== comms.cpp ==================== */
#include "comms/comms.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"
#include "config/HardwareConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/multi_print.h"
#include "log/log.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
#ifdef BOARD_UNO_R4
  static HardwareSerial& bt_serial = Serial1;
#else
  static HardwareSerial& bt_serial = Serial;
#endif

static MultiPrint comms_out(&bt_serial);

#ifdef BOARD_UNO_R4
  static MultiPrint system_out_impl(&bt_serial, &Serial);   // Bluetooth + USB debug
#else
  static MultiPrint system_out_impl(&bt_serial, nullptr);   // Bluetooth only (no USB debug on R3)
#endif

/* =============== PUBLIC API =============== */
namespace Comms {

Print& print  = comms_out;
Print& system = system_out_impl;    

void begin() {
    bt_serial.begin(9600);

#ifdef BOARD_UNO_R4
    /* system_out_impl binds &Serial at static init. Open USB CDC
       unconditionally, not only for the debug mirror. */
    Serial.begin(9600);
#endif

#if COMMS_DEBUG_MIRROR && defined(BOARD_UNO_R4)
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
