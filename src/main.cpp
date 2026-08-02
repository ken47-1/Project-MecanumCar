/* ==================== main.cpp ==================== */
#include "config/Config.h"
#include "config/HardwareConfig.h"
#include "config/DebugConfig.h"

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= COMMUNICATION ========= */
#include "comms/comms.h"

/* ========= INPUT ========= */
#include "input/bluetooth_command_parser.h"
#include "input/input_watchdog.h"

/* ========= CONTROL ========= */
#include "control/mode_manager.h"
#include "control/motor_control.h"
#include "control/motor_ramp.h"
#include "control/motor_fault.h"
#include "control/motor_hardware.h"
#include "control/autonomous_controller.h"

/* ========= SENSORS ========= */
#if ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR
    #include "sensors/ultrasonic.h"
#endif
#if ENABLE_BATTERY_MONITOR
    #include "sensors/battery_voltage.h"
#endif
#if ENABLE_DIRECTIONAL_SCAN
    #include "sensors/directional_scan.h"
#endif

/* ========= SAFETY ========= */
#if ENABLE_OBSTACLE_AVOIDANCE
    #include "safety/obstacle_detection.h"
#endif
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
	#if ENABLE_ULTRASONIC_FRONT || ENABLE_ULTRASONIC_REAR
		Ultrasonic::init();
	#endif
	#if ENABLE_DIRECTIONAL_SCAN
		DirectionalScan::init();
	#endif
	#if ENABLE_OBSTACLE_AVOIDANCE
		ObstacleDetection::init();
	#endif
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

	/* --- Obstacle Detection & Safety --- */
	// Read ultrasonic sensors, apply hysteresis, update proximity flags
	#if ENABLE_OBSTACLE_AVOIDANCE
        ObstacleDetection::update();
	#endif

	// Aggregate all fault states (E-STOP, INPUT_LOSS, CONNECTION_LOSS)
	SafetyManager::update();

    /* --- Battery Monitoring --- */
    #if ENABLE_BATTERY_MONITOR
        BatteryVoltage::report();
    #endif

	/* --- Mode-Specific Logic --- */
	#if ENABLE_AUTONOMOUS_MODE
		AutonomousController::update(input_watchdog);
	#endif

	/* --- Hardware Execution --- */
	MotorRamp::update();
	MotorControl::update();
}