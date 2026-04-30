/*
  Mecanum wheel car ino v2.3.6
  Date: 2023.6.7
  Author: Ajay Huajian
  2023 Copyright(c) ZHIYI Technology Inc. All right reserved
  REFACTORED BY ken471 (using ChatGPT)
*/

// PWM pins for Motor control
const int PWM2A = 11;      // Motor 1 PWM
const int PWM2B = 3;       // Motor 2 PWM
const int PWM0A = 5;       // Motor 3 PWM
const int PWM0B = 6;       // Motor 4 PWM

// L293D Pins for Motor direction control
const int DIR_CLK = 4;     // Clock for shift register
const int DIR_EN = 7;      // Enable pin for L293D
const int DATA = 8;        // Data input for shift register
const int DIR_LATCH = 12;  // Latch for shift register

// Ultrasonic sensor pins (for Obstacle avoidance)
const int Trig = A2;       // A2 as Trig pin for Ultrasonic sensor
const int Echo = A3;       // A3 as Echo pin for Ultrasonic sensor

// Motion directions (values correspond to shift register codes)
const int Forward = 216;   // Forward
const int Backward = 39;       // Backward
const int Left = 116;      // Left
const int Right = 139;     // Right
const int Stop = 0;        // Stop
const int L_turn = 198;    // Turn left
const int R_turn = 57;     // Turn right

// Global speed variable (master)
int speed = 180;

// Default motor speed values (0-255)
int Speed1 = 180;
int Speed2 = 180;
int Speed3 = 180;
int Speed4 = 180;

// Ultrasonic sensor averaging globals
const int NUM_AVERAGE = 3;
int distReadings[NUM_AVERAGE];
int readIndex = 0;
int distance = 0;       // averaged distance
int distanceRaw = 0;    // single reading
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 50; // ms between measurements

// Obstacle Avoidance toggle
bool obstacleAvoidanceEnabled = false;

// Unknown command check
const char* validKeys = 
    "WASD" // Straight
    "QEZC" // Diagonals
    "JL"   // Turns
    "X"    // Stop
    "MO"    // Alt Mode / Obstacle toggle
    "#"    // Speed
    ;

// Manual Override
bool manualOverrideActive = false;
unsigned long lastManualCommandTime = 0;
const unsigned long MANUAL_OVERRIDE_TIMEOUT = 1000; // 1000ms, 1 second

// Stop lock    
bool stopLock = false;
const unsigned long STOP_LOCK_TIME = 200; // 200ms, 0.2 seconds
unsigned long stopLockTimer = 0;

void setup() 
{
    Serial.begin(9600);     // Set the serial port baud rate 9600

    pinMode(DIR_CLK,OUTPUT);
    pinMode(DATA,OUTPUT);
    pinMode(DIR_EN,OUTPUT);
    pinMode(DIR_LATCH,OUTPUT);
    pinMode(PWM0B,OUTPUT);
    pinMode(PWM0A,OUTPUT);
    pinMode(PWM2A,OUTPUT);
    pinMode(PWM2B,OUTPUT);

    pinMode(Trig,OUTPUT);
    pinMode(Echo,INPUT);
}

unsigned long lastJoystickUpdate = 0;  // timestamp of last joystick input
const unsigned long JOYSTICK_TIMEOUT = 150; // ms to wait before stopping

void loop() {
    unsigned long now = millis();

    // Take a measurement every READ_INTERVAL
    if (now - lastReadTime >= READ_INTERVAL) {
        distanceRaw = SR04(); // Take a single reading

        if (distanceRaw != 999) { // Only valid readings
            distReadings[readIndex] = distanceRaw;
            readIndex = (readIndex + 1) % NUM_AVERAGE; 
        }

        // Compute average from valid values
        long sum = 0;
        int count = 0;
        for (int i = 0; i < NUM_AVERAGE; i++) {
            if (distReadings[i] != 999) {
                sum += distReadings[i];
                count++;
            }
        }

        distance = (count > 0) ? sum / count : 999;

        // Print distances
        //Serial.print("Raw: ");
        //Serial.print(distanceRaw);
        //Serial.print("cm | Avg: ");
        //if (distance == 999) {
            //Serial.println("Out of range");
        //}
        //else {
            //Serial.print(distance);
            //Serial.println(" cm");
        //}

        lastReadTime = now;
    }

    // Handle Bluetooth commands
    control_func();

    // Reset manual override if timeout exceeded
    if (manualOverrideActive && (now - lastManualCommandTime > MANUAL_OVERRIDE_TIMEOUT)) {
        manualOverrideActive = false;
    }

    // Reset stop lock if timeout exceeded
    if (stopLock && (now - stopLockTimer > STOP_LOCK_TIME)) {
        stopLock = false;
    }

    // === AUTO STOP ===
    if (manualOverrideActive && (now - lastJoystickUpdate > JOYSTICK_TIMEOUT)) {
        // Stop motors if joystick hasn't updated recently
        Motor(Stop, 0, 0, 0, 0);
        manualOverrideActive = false;
    }

    // Obstacle avoidance only runs if enabled and no manual override
    if (obstacleAvoidanceEnabled && !manualOverrideActive && !stopLock) {
        obstacleAvoidance(now);
    }
}

/* MOTOR CONTROL */
void Motor(int Dir, int S1, int S2, int S3, int S4) {
    analogWrite(PWM2A, S1); // Motor PWM speed regulation
    analogWrite(PWM2B, S2); // Motor PWM speed regulation
    analogWrite(PWM0B, S3); // Motor PWM speed regulation
    analogWrite(PWM0A, S4); // Motor PWM speed regulation
    
    digitalWrite(DIR_LATCH, LOW); // Prepare to write direction
    shiftOut(DATA, DIR_CLK, MSBFIRST, Dir); // Write Dir motion direction value
    digitalWrite(DIR_LATCH, HIGH); // Latch it
}

/* ULTRASONIC SENSOR */
int SR04() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(15);
    digitalWrite(Trig, LOW);

    long duration = pulseIn(Echo, HIGH, 17400); // 17.4ms (~150 cm)
    return (duration == 0) ? 999 : duration / 58; // Convert µs → cm
}

/* OBSTACLE AVOIDANCE STATE*/
enum OA_State {OA_Forward, OA_Back, OA_Turn};
OA_State oaState = OA_Forward;
unsigned long oaLastAction = 0;
const unsigned long OA_BACK_DURATION = 500; // ms to move backward
const unsigned long OA_TURN_DURATION = 500; // ms to turn

/* GLOBAL FOR JOYSTICK */
int joystickMagnitude = 0; // stores latest magnitude input

/* BLUETOOTH CONTROL */
/* MAGNITUDE/ANGLE JOYSTICK + TURN BUTTONS */
void control_func() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') continue;

        // === SPEED CONTROL ===
        if (c == '#') {
            String num = "";
            while (Serial.available() && isDigit(Serial.peek())) {
                num += (char)Serial.read();
            }
            if (num.length() > 0) {
                speed = constrain(num.toInt(), 0, 255);
                Speed1 = Speed2 = Speed3 = Speed4 = speed;
                Serial.print("*S");
                Serial.print(speed);
                Serial.println("*");
            }
            continue;
        }

        // === MAGNITUDE & ANGLE JOYSTICK (Left Side) ===
        if (c == 'R') { // Magnitude
            String magStr = "";
            while (Serial.available() && isDigit(Serial.peek())) {
                magStr += (char)Serial.read();
            }
            int mag = magStr.length() ? magStr.toInt() : 0;
            if (mag > 0) {
                // Store temporary magnitude
                joystickMagnitude = constrain(mag, 0, 255);
            }
            continue;
        }
        if (c == 'A') { // Angle
            String angleStr = "";
            while (Serial.available() && isDigit(Serial.peek())) {
                angleStr += (char)Serial.read();
            }
            int angle = angleStr.length() ? angleStr.toInt() : 0;
            if (angle > 0) {
                // Convert magnitude & angle to forward/strafe PWM
                float rad = angle * 3.1415926 / 180.0;
                int forwardPWM = (int)(joystickMagnitude * cos(rad));
                int strafePWM  = (int)(joystickMagnitude * sin(rad));

                manualOverrideActive = true;
                lastManualCommandTime = millis();

                updateMotors(forwardPWM, strafePWM);
            }
            continue;
        }

        // === TURN BUTTONS (Right Side) ===
        if (c == 'J') { // turn left
            Motor(L_turn, Speed1, Speed2, Speed3, Speed4);
            manualOverrideActive = true;
            lastManualCommandTime = millis();
            continue;
        }
        if (c == 'L') { // turn right
            Motor(R_turn, Speed1, Speed2, Speed3, Speed4);
            manualOverrideActive = true;
            lastManualCommandTime = millis();
            continue;
        }

        // === STOP ===
        if (c == 'X') {
            Motor(Stop, 0, 0, 0, 0);
            manualOverrideActive = false;
            stopLock = true;
            stopLockTimer = millis();
            continue;
        }

        // === OBSTACLE AVOIDANCE TOGGLE ===
        if (c == 'O') {
            obstacleAvoidanceEnabled = !obstacleAvoidanceEnabled;
            Serial.print("Obstacle avoidance ");
            Serial.println(obstacleAvoidanceEnabled ? "ON" : "OFF");
            if (obstacleAvoidanceEnabled) {
                oaState = OA_Forward;
                oaLastAction = millis();
                Motor(Forward, Speed1, Speed2, Speed3, Speed4);
            }
            continue;
        }

        // Unknown command
        if (!strchr(validKeys, c)) {
            Serial.print("Invalid command: ");
            Serial.println(c);
        }
    }
}

/* MODIFIED UPDATE MOTORS FOR JOYSTICK ONLY */
void updateMotors(int forwardPWM, int strafePWM) {
    int m1 = forwardPWM - strafePWM;
    int m2 = forwardPWM + strafePWM;
    int m3 = forwardPWM + strafePWM;
    int m4 = forwardPWM - strafePWM;

    // Scale so largest PWM does not exceed magnitude
    int maxVal = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
    if (maxVal > 255) {
        float scale = 255.0 / maxVal;
        m1 = m1 * scale;
        m2 = m2 * scale;
        m3 = m3 * scale;
        m4 = m4 * scale;
    }

    // Diagonal → use Forward direction for shift register
    int dir = Stop;
    if (forwardPWM != 0 || strafePWM != 0) dir = Forward;

    Motor(dir, abs(m1), abs(m2), abs(m3), abs(m4));
}

/* OBSTACLE AVOIDANCE */
void obstacleAvoidance(unsigned long now) {
    static OA_State lastState = OA_Forward; // Track previous state

    switch (oaState) {
        case OA_Forward:
            if (distance > 20) {
                if (lastState != OA_Forward) {
                    // Move forward normally
                    Motor(Forward, Speed1, Speed2, Speed3, Speed4);
                    lastState = OA_Forward;
                }
            } else {
                // Obstacle detected, start retreat
                oaState = OA_Back;
                oaLastAction = now;
                Motor(Backward, Speed1, Speed2, Speed3, Speed4);
                lastState = OA_Back;
            }
            break;

        case OA_Back:
            if (now - oaLastAction >= OA_BACK_DURATION) {
                // Finished retreating, start turning
                oaState = OA_Turn;
                oaLastAction = now;
                Motor(L_turn, Speed1, Speed2, Speed3, Speed4); // Or sliding turn
                lastState = OA_Turn;
            }
            break;

        case OA_Turn:
            if (now - oaLastAction >= OA_TURN_DURATION) {
                // Finished turning, resume forward movement
                oaState = OA_Forward;
                Motor(Forward, Speed1, Speed2, Speed3, Speed4);
                lastState = OA_Forward;
            }
            break;
    }
}