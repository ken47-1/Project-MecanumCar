/* ==================== main.cpp ==================== */

/* =============== INCLUDES =============== */
/* ============ PROJECT ============ */
#include "config/Config.h"
#include "comms/comms.h"
#include "input/bluetooth_command_parser.h"
#include "input/input_watchdog.h"
#include "control/mode_manager.h"
#include "control/motor_control.h"
#include "control/motor_ramp.h"
#include "control/motor_fault.h"
#include "control/motor_hardware.h"
#include "control/autonomous_controller.h"
#include "sensors/sensors.h"
#include "sensors/directional_scan.h"
#include "safety/obstacle_detection.h"
#include "safety/safety_manager.h"

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
    BluetoothCommandParser::handle(input_watchdog);

    /* --- Safety & Watchdog Ticks --- */
    input_watchdog.update();
    
    /* --- HC-05 Connection Status (if enabled) --- */
    #if ENABLE_HC05_STATE_PIN
        if (!Comms::is_connected()) {
            SafetyManager::set_connection_loss(true);
        } else {
            SafetyManager::set_connection_loss(false);
        }
    #endif

    ObstacleDetection::update();
    SafetyManager::update();

    /* --- Mode-Specific Logic --- */
    AutonomousController::update(input_watchdog);

    /* --- Hardware Execution --- */
    MotorRamp::update();
    MotorControl::update();
}