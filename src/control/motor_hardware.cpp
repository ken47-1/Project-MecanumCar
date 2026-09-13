/* ==================== motor_hardware.cpp ==================== */
#include "control/motor_hardware.h"

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= CONTROL ========= */
#include "control/motor_fault.h"

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== PUBLIC API =============== */
bool MotorHardware::init() {
    if (!_shield.begin()) {
        MotorFault::trigger(MotorFaultReason::SHIELD_NOT_FOUND);
        return false;
    }

    _fl = _shield.getMotor(1);
    _fr = _shield.getMotor(4);
    _rl = _shield.getMotor(2);
    _rr = _shield.getMotor(3);

    if (!_fl || !_fr || !_rl || !_rr) {
        MotorFault::trigger(MotorFaultReason::INTERNAL_ERROR);
        return false;
    }

    Comms::system.println(F("MotorHardware INIT"));
    if (PWM_MAX == 4095) {
        Comms::system.println(F("- PWM: 12-bit (AFMS V2)"));
    } else if (PWM_MAX == 255) {
        Comms::system.println(F("- PWM: 8-bit (AFMS V1)"));
    } else {
        Comms::system.print(F("- PWM_MAX: "));
        Comms::system.println(PWM_MAX);
    }

    return true;
}

bool MotorHardware::ready() const {
    return _fl && _fr && _rl && _rr;
}

Adafruit_DCMotor* MotorHardware::get(MotorId id) {
    switch (id) {
        case MotorId::FL: return _fl;
        case MotorId::FR: return _fr;
        case MotorId::RL: return _rl;
        case MotorId::RR: return _rr;
    }
    return nullptr;
}

void MotorHardware::release_all() {
    if (_fl) {
        _fl->run(RELEASE);
    }
    
    if (_fr) {
        _fr->run(RELEASE);
    }

    if (_rl) {
        _rl->run(RELEASE);
    }
    
    if (_rr) {
        _rr->run(RELEASE);
    }
}