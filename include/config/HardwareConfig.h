/* ==================== HardwareConfig.h ==================== */
#pragma once

/* =============== INCLUDES =============== */

/* ============ CORE ============ */
#include <stdint.h>
#include <Arduino.h>

/* =============== FEATURE FLAGS =============== */
/* Set to 1 if the hardware is physically installed, 0 if not. 
   Disabling unused features saves flash space and prevents false faults. */

#define ENABLE_HC05_STATE_PIN   0   // Set to 1 if using HC-05 with STATE pin wired.
#define ENABLE_ULTRASONIC_FRONT 1   // Front HC-SR04
#define ENABLE_ULTRASONIC_REAR  1   // Rear HC-SR04
#define ENABLE_SERVO            1   // Servo for directional scan
#define ENABLE_BATTERY_MONITOR  0   // 0-25V voltage sensor
#define ENABLE_ENCODERS         0   // H206 optical encoders

/* =============== COMMS =============== */

/* ============ BLUETOOTH ============ */
/* Bluetooth: R3 uses Serial (pins 0/1), R4 uses Serial1 (pins 0/1)
   R3: Disconnect Bluetooth when uploading (pins shared with USB)
   R4: Upload with Bluetooth connected (Serial1 is independent)
*/

#if ENABLE_HC05_STATE_PIN
    constexpr uint8_t BT_STATE_PIN = 2;
#endif

/* =============== SENSORS =============== */

/* ============ ULTRASONIC ============ */
/* TRIG pins must be digital outputs. ECHO pins must be digital inputs. */
constexpr uint8_t SR04_FRONT_TRIG_PIN = 11;
constexpr uint8_t SR04_FRONT_ECHO_PIN = 12;

constexpr uint8_t SR04_REAR_TRIG_PIN  = 8;
constexpr uint8_t SR04_REAR_ECHO_PIN  = 9;

/* ============ BATTERY MONITORING (REV 2) ============ */
constexpr uint8_t BATTERY_SENSOR_PIN   = A0;
/* Formula: (R1 + R2) / R2. For a 30k/7.5k divider, ratio = 5.0. */
constexpr float   BATTERY_DIVIDER_RATIO = 5.0f; // 25V → 5V at ADC pin

constexpr float BATTERY_VOLTAGE_MIN = 6.0f; // Matches CRITICAL in Config.h
constexpr float BATTERY_VOLTAGE_MAX = 8.4f; // 2S Li-Ion fully charged

static_assert(BATTERY_VOLTAGE_MIN < BATTERY_VOLTAGE_MAX, 
              "BATTERY_VOLTAGE_MIN must be less than BATTERY_VOLTAGE_MAX");

/* =============== ACTUATORS =============== */

/* ============ SERVO ============ */
constexpr uint8_t SCAN_SERVO_PIN = 10;

#define SERVO_MIRRORED  1

#if SERVO_MIRRORED
/* Mirrored mounting (servo facing down) */
constexpr int SERVO_LEFT        = 0;
constexpr int SERVO_FRONT_LEFT  = 45;
constexpr int SERVO_CENTER      = 90;
constexpr int SERVO_FRONT_RIGHT = 135;
constexpr int SERVO_RIGHT       = 180;
#else
/* Normal mounting (servo facing up) */
constexpr int SERVO_LEFT        = 180;
constexpr int SERVO_FRONT_LEFT  = 135;
constexpr int SERVO_CENTER      = 90;
constexpr int SERVO_FRONT_RIGHT = 45;
constexpr int SERVO_RIGHT       = 0;
#endif

/* ============ ENCODERS (REV 3) ============ */
/* On R3, these must be PCINT-capable pins (D0-D7). 
   On R4, any digital pin works via AGTimer polling. */
constexpr uint8_t ENCODER_FL_PIN = 7;
constexpr uint8_t ENCODER_FR_PIN = 6;
constexpr uint8_t ENCODER_RL_PIN = 5;
constexpr uint8_t ENCODER_RR_PIN = 4;
