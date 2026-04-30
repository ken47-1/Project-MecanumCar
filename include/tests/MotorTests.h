/* ==================== MotorControlTests.h ==================== */
#pragma once

enum class MotorTestId {
    NONE,
    ALL_MOTORS,
    PWM_SINGLE,
    PWM_ALL
};

void run_motor_test(MotorTestId test);