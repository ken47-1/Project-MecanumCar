/* ==================== main.cpp ==================== */

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/Comms.h"
#include "input/BluetoothCommandParser.h"
#include "input/InputWatchdog.h"
#include "control/ModeManager.h"
#include "control/MotorControl.h"
#include "control/MotorRamp.h"
#include "control/MotorFault.h"
#include "control/MotorHardware.h"
#include "control/AutonomousController.h"
#include "sensors/Sensors.h"
#include "sensors/DirectionalScan.h"
#include "safety/ObstacleDetection.h"
#include "safety/SafetyManager.h"

/* ============ CORE ============ */
#include <Wire.h>
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
/* ============ STATIC VARS ============ */
static MotorHardware motor_hw;
static InputWatchdog input_watchdog(INPUT_WATCHDOG_TIMEOUT_MS);

/* =============== LIFECYCLE =============== */
/* ============ SETUP ============ */
void setup() {
    /* --- Comms & Bus --- */
    Comms::begin();
    Wire.begin();

    /* --- Control System --- */
    MotorFault::init();
    motor_hw.init();
    MotorControl::init(motor_hw);
    ModeManager::init();

    /* --- Navigation & Safety --- */
    Sensors::init();
    DirectionalScan::init();
    ObstacleDetection::init();
    SafetyManager::init();

    /* --- Watchdog Activation --- */
    input_watchdog.enable(true);
    input_watchdog.feed();   // Prevent false INPUT_LOSS at startup
}

/* ============ LOOP ============ */
void loop() {
    /* --- Input Processing --- */
    /* System commands work in any mode; Manual intent is applied if in MANUAL */
    BluetoothCommandParser::handle(input_watchdog);

    /* --- Safety & Watchdog Ticks --- */
    /* Watchdog push state to SafetyManager; Sensors update zones */
    input_watchdog.update();
    ObstacleDetection::update();
    SafetyManager::update();

    /* --- Mode-Specific Logic --- */
    /* Only acts if ModeManager is in AUTO; Handles its own watchdog feeding */
    AutonomousController::update(input_watchdog);

    /* --- Hardware Execution --- */
    /* Processes ramping curves and writes final PWM to shield */
    MotorRamp::update();
    MotorControl::update();
}