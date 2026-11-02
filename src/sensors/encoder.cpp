/* ==================== encoder.cpp ==================== */
#include "sensors/encoder.h"

#if ENABLE_ENCODERS

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

/* ========= SAFETY ========= */
#include "safety/system_fault.h"

/* ============ THIRD-PARTY ============ */
#ifdef BOARD_UNO_R4
    #include "AGTimerR4.h"
#endif

/* ============ CORE ============ */
#include <Arduino.h>

/* =============== INTERNAL STATE =============== */
/* ============ PIN MAP ============ */
static const uint8_t PINS[4] = {
    ENCODER_FL_PIN,
    ENCODER_FR_PIN,
    ENCODER_RL_PIN,
    ENCODER_RR_PIN
};

/* ============ COUNTERS ============ */
static volatile int32_t  total[4]          = {0, 0, 0, 0};
static volatile uint32_t last_edge_us[4]   = {0, 0, 0, 0};
static volatile uint32_t period_us[4]      = {0, 0, 0, 0};
static volatile int8_t   dir_state[4]      = {0, 0, 0, 0};
static volatile uint8_t  last_pin_state[4] = {1, 1, 1, 1};

/* =============== INTERNAL HELPERS =============== */
static inline void accept_edge(uint8_t idx) {
    uint32_t now  = micros();
    uint32_t prev = last_edge_us[idx];

    if (prev != 0 && (now - prev) < ENCODER_DEBOUNCE_US) {
        return;
    }

    last_edge_us[idx] = now;

    if (prev == 0) {
        return;
    }

    period_us[idx] = now - prev;
    total[idx]    += dir_state[idx];
}

/* =============== ISR =============== */
#ifdef BOARD_UNO_R4

static void poll_all() {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t s = digitalRead(PINS[i]);
        if (s != last_pin_state[i]) {
            last_pin_state[i] = s;
            if (s == HIGH) accept_edge(i);
        }
    }
}

#else

static volatile uint8_t last_pind = 0;

ISR(PCINT2_vect) {
    uint8_t pind    = PIND;
    uint8_t changed = pind ^ last_pind;
    last_pind       = pind;

    if ((changed & (1 << ENCODER_FL_PIN)) && (pind & (1 << ENCODER_FL_PIN))) {
        accept_edge(0);
    }
    if ((changed & (1 << ENCODER_FR_PIN)) && (pind & (1 << ENCODER_FR_PIN))) {
        accept_edge(1);
    }
    if ((changed & (1 << ENCODER_RL_PIN)) && (pind & (1 << ENCODER_RL_PIN))) {
        accept_edge(2);
    }
    if ((changed & (1 << ENCODER_RR_PIN)) && (pind & (1 << ENCODER_RR_PIN))) {
        accept_edge(3);
    }
}

#endif

/* =============== PUBLIC API =============== */
namespace Encoder {

void init() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PINS[i], INPUT_PULLUP);
        last_pin_state[i] = digitalRead(PINS[i]);
    }

#ifdef BOARD_UNO_R4
    AGTimer.init(ENCODER_POLL_PERIOD_US, poll_all);
    if (!AGTimer.start()) {
        Comms::system.println(F("Encoder: AGTimer failed to start"));
        SystemFault::trigger(SystemFaultReason::INTERNAL_ERROR);
        return;
    }
#else
    last_pind = PIND;

    PCMSK2 |= (1 << ENCODER_FL_PIN)
           |  (1 << ENCODER_FR_PIN)
           |  (1 << ENCODER_RL_PIN)
           |  (1 << ENCODER_RR_PIN);
    PCICR  |= (1 << PCIE2);
#endif

    Comms::system.println(F("Encoder INIT"));
}

void reset() {
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
        total[i]          = 0;
        last_edge_us[i]   = 0;
        period_us[i]      = 0;
        dir_state[i]      = 0;
        last_pin_state[i] = digitalRead(PINS[i]);
    }
    interrupts();
}

void set_direction(uint8_t idx, int8_t dir) {
    if (idx >= 4) {
        return;
    }
    if (dir == dir_state[idx]) {
        return;
    }

    noInterrupts();
    dir_state[idx]    = dir;
    last_edge_us[idx] = 0;
    period_us[idx]    = 0;
    interrupts();
}

int32_t get_count(uint8_t idx) {
    if (idx >= 4) {
        return 0;
    }
    noInterrupts();
    int32_t v = total[idx];
    interrupts();
    return v;
}

float get_rpm(uint8_t idx) {
    if (idx >= 4) {
        return 0.0f;
    }

    noInterrupts();
    uint32_t last   = last_edge_us[idx];
    uint32_t period = period_us[idx];
    int8_t   dir    = dir_state[idx];
    interrupts();

    if (last == 0 || period == 0 || dir == 0) {
        return 0.0f;
    }

    if (micros() - last > ENCODER_STALL_TIMEOUT_US) {
        return 0.0f;
    }

    float magnitude = 60000000.0f / ((float)period * (float)ENCODER_TICKS_PER_REV);
    return magnitude * (float)dir;
}

}

#endif // ENABLE_ENCODERS