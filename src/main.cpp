/* ==================== main.cpp ==================== */

/* =============== INCLUDES =============== */

/* ============ CONFIG ============ */
#include "config/Config.h"
#include "config/HardwareConfig.h"
#include "config/DebugConfig.h"

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= INPUT ========= */
#include "input/input_watchdog.h"
#include "input/bluetooth_command_parser.h"
#include "input/bluetooth_speed_authority.h"

/* ========= CONTROL ========= */
#include "control/mode_manager.h"
#include "control/motor_control.h"
#include "control/motor_ramp.h"
#include "control/motor_fault.h"
#include "control/motor_hardware.h"
#include "control/autonomous_controller.h"

/* ========= SENSORS ========= */
#include "sensors/ultrasonic.h"
#include "sensors/battery_voltage.h"
#include "sensors/directional_scan.h"
#include "sensors/encoder.h"

/* ========= SAFETY ========= */
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

    /* Seed the PRNG from an unconnected analog pin. */
    analogRead(A1);
    randomSeed(analogRead(A1));

    /* --- Control System --- */
    MotorFault::init();
    motor_hw.init();
    MotorControl::init(motor_hw);
    ModeManager::init();

    /* --- Navigation & Safety --- */
    Ultrasonic::init();
    Encoder::init();
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

    /* --- Speed feedback keepalive --- */
    BluetoothSpeedAuthority::feedback_tick();
    
    /* --- HC-05 Connection Status (if enabled) --- */
    #if ENABLE_HC05_STATE_PIN
        if (!Comms::is_connected()) {
            SafetyManager::set_connection_loss(true);
        } else {
            SafetyManager::set_connection_loss(false);
        }
    #endif

    /* --- Obstacle Detection & Safety --- */
    // Read ultrasonic sensors, apply hysteresis, update proximity flags
    ObstacleDetection::update();

    // Aggregate all fault states (E-STOP, INPUT_LOSS, CONNECTION_LOSS)
    SafetyManager::update();

    /* --- Battery Monitoring --- */
    #if ENABLE_BATTERY_MONITOR
        BatteryVoltage::report();
    #endif

    /* --- Mode-Specific Logic --- */
    AutonomousController::update(input_watchdog);

    /* --- Hardware Execution --- */
    MotorRamp::update();
    MotorControl::update();
}
