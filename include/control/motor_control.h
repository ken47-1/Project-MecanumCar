/* ==================== motor_control.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/motor_hardware.h"
#include "control/motion_command.h"

/* =============== API =============== */
namespace MotorControl {
    void init(MotorHardware& hw);
    void hard_stop();
    void apply_command(const MotionCommand& cmd);
    void update();
}