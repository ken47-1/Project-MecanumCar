/* ==================== MotorControl.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "control/MotorHardware.h"
#include "control/MotionCommand.h"

/* =============== API =============== */
namespace MotorControl {
    void init(MotorHardware& hw);
    void hard_stop();
    void apply_command(const MotionCommand& cmd);
    void update();
}