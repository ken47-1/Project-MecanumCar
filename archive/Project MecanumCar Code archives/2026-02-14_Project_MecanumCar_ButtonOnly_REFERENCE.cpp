#include <Arduino.h>

// FILE LOCATION REQUIREMENT:
// This file must live OUTSIDE src/ or it will be compiled.

/* =========================================================
ARCHIVED — DO NOT COMPILE
Known-good button-based control (pre-mecanum-math)

Purpose:
- Motor direction reference
- Spin behavior reference
- Debug fallback

This file is intentionally NOT used by the build.
========================================================= */

/*
    Mecanum wheel car ino v2.3.6 (HACKED)
    Author: Ajay Huajian
    2023 Copyright(c) ZHIYI Technology Inc. All right reserved

    Fixed and Modified by Ken471 (with Assistance from OpenAI ChatGPT and Google Gemini)
*/

#define ENABLE_MOTOR_PWM_TEST 0
#define ENABLE_MOTOR_BITMASK_TEST 0

/* --- PIN CONFIGURATION (L293D) --- */
// Arduino PWM -> L293D Enable Pins (Motors)
constexpr uint8_t PWM2A = 11;
constexpr uint8_t PWM2B = 3;
constexpr uint8_t PWM0A = 5;
constexpr uint8_t PWM0B = 6;

// Logical wheel -> physical PWM pin mapping
constexpr uint8_t PWM_FrontLeft = PWM2A;
constexpr uint8_t PWM_FrontRight = PWM0A;
constexpr uint8_t PWM_RearLeft = PWM2B;
constexpr uint8_t PWM_RearRight = PWM0B;

// 74HC595 Shift Register -> L293D Motor Driver Pins
// DIR_EN is active LOW
constexpr uint8_t DIR_CLK   = 4;  // Shift Clock
constexpr uint8_t DIR_EN    = 7;  // Output Enable (Active LOW)
constexpr uint8_t DATA      = 8;  // Serial Data
constexpr uint8_t DIR_LATCH = 12; // Storage register Latch

/* --- MOTOR DIRECTIONS (L293D Shift register codes) --- */
constexpr uint8_t DIR_FORWARD  = 216; // 0b11011000
constexpr uint8_t DIR_BACKWARD = 39;  // 0b00100111
constexpr uint8_t DIR_LEFT     = 116; // 0b01110100
constexpr uint8_t DIR_RIGHT    = 139; // 0b10001011
constexpr uint8_t DIR_STOP     = 0;   // 0b00000000
constexpr uint8_t DIR_SPIN_L = 198;   // 0b11000110
constexpr uint8_t DIR_SPIN_R = 57;    // 0b00111001

/* --- SPEED SETTINGS --- */
// Global speed (0-255)
uint8_t speed = 255;

// Speed control (0-255)
constexpr uint8_t SPEED_MIN = 0;
constexpr uint8_t SPEED_MAX = 255;
constexpr uint8_t SPEED_WARN_THRESHOLD = 120;

// Step sizes
uint8_t speedStep = 10;          
constexpr uint8_t STEP_FINE   = 5;
constexpr uint8_t STEP_NORMAL = 10;
constexpr uint8_t STEP_ROUGH  = 20;

/* TIMEOUT WATCHDOG CONFIGURATION */
constexpr unsigned long TIMEOUT_THRESHOLD = 100; // (in ms) Timeout Threshold
unsigned long lastCmdTime = 0;                   // (in ms) Time since last command

/* --- CONTROL PROTOCOL REFERENCE --- */
/*  FORMAT: NAME [PREFIX]: COMMANDS

    COMMANDS (App -> Car):
    - Movement [None]: WASD (Straight), QEZC (Diagonal), JL (Turn)
    - Speed    [%]:    %+ (Up), %- (Down), %F (Fine), %N (Normal), %R (Rough)
    - System   [None]: ! (Stop/Emergency Stop)
   
    FEEDBACK (Car -> App):
    - Speed Value [G]: *G[0-255]* -> (Update Gauge)
    - Speed Step  [%]: *%[Mode]*  -> (Update Textbox: Fine/Normal/Rough)
*/

/* --- RAW MOTOR CONTROL --- */
void driveRawMotors(uint8_t dir,
                    uint8_t speedFL, uint8_t speedFR,
                    uint8_t speedRL, uint8_t speedRR) {
    analogWrite(PWM_FrontLeft, speedFL);
    analogWrite(PWM_FrontRight, speedFR);
    analogWrite(PWM_RearLeft, speedRL);
    analogWrite(PWM_RearRight, speedRR);

    digitalWrite(DIR_LATCH, LOW);
    shiftOut(DATA, DIR_CLK, MSBFIRST, dir);
    digitalWrite(DIR_LATCH, HIGH);
}

/* --- MOTOR PWM TEST --- */
#if ENABLE_MOTOR_PWM_TEST
void testMotorPWM() {
    lastCmdTime = 0; // Disable watchdog
    Serial.println("Testing PWM_FrontLeft (PWM2A)");
    analogWrite(PWM_FrontLeft, 255);
    delay(1000);
    analogWrite(PWM_FrontLeft, 0);
    delay(500);

    Serial.println("Testing PWM_FrontRight (PWM0A)");
    analogWrite(PWM_FrontRight, 255);
    delay(1000);
    analogWrite(PWM_FrontRight, 0);
    delay(500);

    Serial.println("Testing PWM_RearLeft (PWM2B)");
    analogWrite(PWM_RearLeft, 255);
    delay(1000);
    analogWrite(PWM_RearLeft, 0);
    delay(500);

    Serial.println("Testing PWM_RearRight (PWM0B)");
    analogWrite(PWM_RearRight, 255);
    delay(1000);
    analogWrite(PWM_RearRight, 0);
    delay(500);

    digitalWrite(DIR_LATCH, LOW);
    shiftOut(DATA, DIR_CLK, MSBFIRST, DIR_FORWARD);
    digitalWrite(DIR_LATCH, HIGH);

    Serial.println("TEST FINISHED");
}
#endif

/* --- BLUETOOTH HANDLER --- */
void handleBluetooth() {
    static String serialBuffer = "";
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        // Trim Newlines (\n) and Carriage returns (\r)
        if (c == '\n' || c == '\r') continue; 

        // 1. STOP COMMAND
        if (c == '!') {
            driveRawMotors(DIR_STOP, 0, 0, 0, 0);
            lastCmdTime = 0;
            serialBuffer = ""; 
            return;
        }

        // 2. BUTTON LOGIC
        if (c == 'W' || c == 'S' || c == 'A' || c == 'D' || // Straight/Strafes
            c == 'Q' || c == 'E' || c == 'Z' || c == 'C' || // Diagonals
            c == 'J' || c == 'L') {                         // Turns

                serialBuffer = "";
                switch (c) {
                    // Straight
                    case 'W': driveRawMotors(DIR_FORWARD,  speed, speed, speed, speed); break; // Forward
                    case 'S': driveRawMotors(DIR_BACKWARD, speed, speed, speed, speed); break; // Backward

                    // Strafes
                    case 'A': driveRawMotors(DIR_LEFT,     speed, speed, speed, speed); break; // Strafe Left
                    case 'D': driveRawMotors(DIR_RIGHT,    speed, speed, speed, speed); break; // Strafe Right
                    
                    // Diagonals
                    case 'Q': driveRawMotors(DIR_FORWARD,  0, speed, speed, 0); break; // Forward-Left
                    case 'E': driveRawMotors(DIR_FORWARD,  speed, 0, 0, speed); break; // Forward-Right
                    case 'Z': driveRawMotors(DIR_BACKWARD, speed, 0, 0, speed); break; // Backward-Left
                    case 'C': driveRawMotors(DIR_BACKWARD, 0, speed, speed, 0); break; // Backward-Right
                    
                    // Turns
                    case 'J': driveRawMotors(DIR_SPIN_L, speed, speed, speed, speed); break; // Spin Left (Counterclockwise)
                    case 'L': driveRawMotors(DIR_SPIN_R, speed, speed, speed, speed); break; // Spin Right (Clockwise)
                }
                lastCmdTime = millis();
                continue;
        }

        // 3. SPEED LOGIC
        if (c == '%' && serialBuffer != "%") { serialBuffer = "%"; continue;}
        if (serialBuffer == "%" && c != '%') {
            char action = c;
            uint8_t oldSpeedStep = speedStep;

            switch (action) {
                case '+': speed = constrain(speed + speedStep, SPEED_MIN, SPEED_MAX); break; // Increase speed by speedStep
                case '-': speed = constrain(speed - speedStep, SPEED_MIN, SPEED_MAX); break; // Increase speed by speedStep
                
                case 'R': speedStep = STEP_ROUGH; break;  // set Step size to ROUGH
                case 'N': speedStep = STEP_NORMAL; break; // set Step size to NORMAL
                case 'F': speedStep = STEP_FINE;   break; // set Step size to FINE
            }

            // Send gauge update
            Serial.print("*G"); Serial.print(speed); Serial.println("*");
            
            if (speed < SPEED_WARN_THRESHOLD) { Serial.print("SPEED: Too low! "); Serial.print("("); Serial.print(speed); Serial.println(")"); }
            
            // Send step size text ONLY if it is changed
            if (speedStep != oldSpeedStep) {
                Serial.print("*%"); 
                if (speedStep == STEP_FINE)        Serial.print("Fine");
                else if (speedStep == STEP_NORMAL) Serial.print("Normal");
                else if (speedStep == STEP_ROUGH)  Serial.print("Rough");
                Serial.println("*");
            }
            
            serialBuffer = "";
            continue;
        }

        // 4. JUNK CLEARING
        if (serialBuffer.length() > 20) serialBuffer = "";
    }
}

/* --- TIMEOUT WATCHDOG --- */
void runWatchdog() {
  // If lastCmdTime is 0, The car is already stopped or in E-Stop mode
  if (lastCmdTime == 0) return; 

  // Check if Current Time (Milliseconds) - Last command time > Timeout Threshold
  if (millis() - lastCmdTime > TIMEOUT_THRESHOLD) {
    driveRawMotors(DIR_STOP, 0, 0, 0, 0);
    lastCmdTime = 0; // Mark last command time as stopped
    Serial.println("WATCHDOG: Signal lost!");
    Serial.println("WATCHDOG: E-Stop engaged.");
  }
}

/* --- SETUP --- */
void setup() {
    Serial.begin(9600);

    pinMode(DIR_CLK, OUTPUT);
    pinMode(DATA, OUTPUT);
    pinMode(DIR_EN, OUTPUT);
    pinMode(DIR_LATCH, OUTPUT);
    pinMode(PWM0A, OUTPUT);
    pinMode(PWM0B, OUTPUT);
    pinMode(PWM2A, OUTPUT);
    pinMode(PWM2B, OUTPUT);

    digitalWrite(DIR_EN, LOW); // Enable Motor driver

    /* NOTES */
    // DIR_EN is Active LOW

    /* TESTS */
    // Motor PWM test
    #if ENABLE_MOTOR_PWM_TEST
    testMotorPWM();
    #endif

    // Motor Bitmask test
    #if ENABLE_MOTOR_BITMASK_TEST
    testMotorBitmask();
    #endif
}

/* --- LOOP --- */
void loop() {
    handleBluetooth();
    runWatchdog();
}