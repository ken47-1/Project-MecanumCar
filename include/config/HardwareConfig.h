/* ==================== HardwareConfig.h ==================== */
#pragma once

#include <stdint.h>
#include <Arduino.h>

/* =============== PHYSICAL HARDWARE PRESENCE =============== */
/* These flags indicate what sensors/modules are physically installed. */

#define ENABLE_HC05_STATE_PIN      0   // HC-05 STATE pin (enable for HC-05)
#define ENABLE_ULTRASONIC_FRONT    1   // Front HC-SR04 installed
#define ENABLE_ULTRASONIC_REAR     1   // Rear HC-SR04 installed
#define ENABLE_SERVO               1   // Servo for directional scan installed
#define ENABLE_BATTERY_MONITOR     1   // 0-25V voltage sensor installed (Rev 2)
#define ENABLE_ENCODERS            0   // H206 encoders installed (Rev 3)

/* =============== BLUETOOTH =============== */
/* Bluetooth: R3 uses Serial (pins 0/1), R4 uses Serial1 (pins 0/1)
   R3: Disconnect Bluetooth when uploading (pins shared with USB)
   R4: Upload with Bluetooth connected (Serial1 is independent)
*/

#if ENABLE_HC05_STATE_PIN
    constexpr uint8_t BT_STATE_PIN = 2;
#endif

/* =============== ULTRASONIC SENSORS =============== */

/* ============ FRONT ============ */
constexpr uint8_t SR04_FRONT_TRIG_PIN = 11;
constexpr uint8_t SR04_FRONT_ECHO_PIN = 12;

/* ============ REAR ============ */
constexpr uint8_t SR04_REAR_TRIG_PIN = 8;
constexpr uint8_t SR04_REAR_ECHO_PIN = 9;

/* =============== SERVO =============== */

/* --- Pin --- */
constexpr uint8_t SCAN_SERVO_PIN = 10;

/* --- Scan Angles (degrees) --- */
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

/* =============== BATTERY MONITORING (REV 2) =============== */
/* 0-25V voltage sensor module */

/* --- Pin --- */
constexpr uint8_t BATTERY_SENSOR_PIN = A0;

/* --- Voltage Divider --- */
constexpr float BATTERY_DIVIDER_RATIO = 5.0f;   // 25V → 5V at ADC pin

/* --- Operating Range --- */
constexpr float BATTERY_VOLTAGE_MIN = 6.0f;     // Matches CRITICAL in Config.h
constexpr float BATTERY_VOLTAGE_MAX = 8.4f;     // 2S Li-Ion fully charged

/* =============== ENCODERS (REV 3) =============== */
/*
constexpr uint8_t ENCODER_FL_PIN = 7;
constexpr uint8_t ENCODER_FR_PIN = 6;
constexpr uint8_t ENCODER_RL_PIN = 5;
constexpr uint8_t ENCODER_RR_PIN = 4;
*/