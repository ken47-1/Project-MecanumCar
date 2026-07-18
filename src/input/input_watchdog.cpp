/* ==================== input_watchdog.cpp ==================== */
#include "input/input_watchdog.h"

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "safety/safety_manager.h"

/* ============ CORE ============ */
#include <Arduino.h>

#if ENABLE_INPUT_WATCHDOG

InputWatchdog::InputWatchdog(uint32_t timeout_ms)
    : _timeout_ms(timeout_ms),
      _last_seen(0),
      _armed(false),
      _enabled(false) {}

void InputWatchdog::feed() {
    _last_seen = millis();
    _armed = true;
    
    // Feed results in immediate clearing of input loss flag
    SafetyManager::set_input_loss(false);
}

void InputWatchdog::reset() {
    _last_seen = millis();
    _armed = false;
    SafetyManager::set_input_loss(false);
}

void InputWatchdog::enable(bool state) {
    _enabled = state;
    if (_enabled) {
        _last_seen = millis();
        _armed = true;
    } else {
        _armed = false;
        SafetyManager::set_input_loss(false);
    }
}

void InputWatchdog::update() {
    if (!_enabled || !_armed) return;

    /* 
       Only push 'true' to SafetyManager. 
       Let the 'feed()' function handle setting it to false 
       to prevent logic racing.
    */
    if (is_expired()) {
        SafetyManager::set_input_loss(true);
    }
}

bool InputWatchdog::is_expired() const {
    if (!_enabled || !_armed) return false;
    return (millis() - _last_seen) > _timeout_ms;
}

uint32_t InputWatchdog::_now() const {
    return millis();
}

#endif