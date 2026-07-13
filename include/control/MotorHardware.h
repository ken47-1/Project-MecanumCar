/* ==================== MotorHardware.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ THIRD-PARTY ============ */
#include <Adafruit_MotorShield.h>

/* =============== TYPES =============== */
enum class MotorId {
    FL, FR,
    RL, RR
};

/* =============== API =============== */
class MotorHardware {
public:
    bool init();
    bool ready() const;

    Adafruit_DCMotor* get(MotorId id);
    void release_all();

private:
    Adafruit_MotorShield _shield;
    Adafruit_DCMotor* _fl = nullptr;
    Adafruit_DCMotor* _fr = nullptr;
    Adafruit_DCMotor* _rl = nullptr;
    Adafruit_DCMotor* _rr = nullptr;
};