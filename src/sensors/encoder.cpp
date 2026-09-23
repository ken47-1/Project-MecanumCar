/* ==================== encoder.cpp ==================== */
#include "sensors/encoder.h"

#if ENABLE_ENCODERS

/* =============== INCLUDES =============== */

/* ============ PROJECT ============ */

/* ========= COMMS ========= */
#include "comms/comms.h"

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
static volatile int32_t  total[4]        = {0, 0, 0, 0};
static volatile uint32_t last_edge_us[4] = {0, 0, 0, 0};
static volatile uint32_t period_us[4]    = {0, 0, 0, 0};
static volatile int8_t   dir_state[4]    = {0, 0, 0, 0};

/* =============== INTERNAL HELPERS =============== */
static inline void count_pulse(uint8_t idx) {
    uint32_t now  = micros();
    uint32_t prev = last_edge_us[idx];
    last_edge_us[idx] = now;

    /* First pulse after a reset has no reference. */
    if (prev == 0) {
        return;
    }

    period_us[idx] = now - prev;
    total[idx]    += dir_state[idx];
}

/* =============== ISR =============== */
#ifdef BOARD_UNO_R4

static void isr_fl() { count_pulse(0); }
static void isr_fr() { count_pulse(1); }
static void isr_rl() { count_pulse(2); }
static void isr_rr() { count_pulse(3); }

#else

static volatile uint8_t last_pind = 0;

ISR(PCINT2_vect) {
    uint8_t pind    = PIND;
    uint8_t changed = pind ^ last_pind;
    last_pind       = pind;

    /* Rising edge only: pin was LOW, is now HIGH. */
    if ((changed & (1 << ENCODER_FL_PIN)) && (pind & (1 << ENCODER_FL_PIN))) {
        count_pulse(0);
    }
    if ((changed & (1 << ENCODER_FR_PIN)) && (pind & (1 << ENCODER_FR_PIN))) {
        count_pulse(1);
    }
    if ((changed & (1 << ENCODER_RL_PIN)) && (pind & (1 << ENCODER_RL_PIN))) {
        count_pulse(2);
    }
    if ((changed & (1 << ENCODER_RR_PIN)) && (pind & (1 << ENCODER_RR_PIN))) {
        count_pulse(3);
    }
}

#endif

/* =============== PUBLIC API =============== */
namespace Encoder {

void init() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PINS[i], INPUT_PULLUP);
    }

#ifdef BOARD_UNO_R4
    attachInterrupt(digitalPinToInterrupt(PINS[0]), isr_fl, RISING);
    attachInterrupt(digitalPinToInterrupt(PINS[1]), isr_fr, RISING);
    attachInterrupt(digitalPinToInterrupt(PINS[2]), isr_rl, RISING);
    attachInterrupt(digitalPinToInterrupt(PINS[3]), isr_rr, RISING);
#else
    /* Capture current state so the first ISR does not report a phantom edge. */
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
        total[i]        = 0;
        last_edge_us[i] = 0;
        period_us[i]    = 0;
        dir_state[i]    = 0;
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
    /* Drop the reference. The next pulse starts a fresh period measurement
       instead of comparing across the direction change. */
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

    /* No pulse for 200 ms means the wheel is stopped. */
    if (micros() - last > 200000UL) {
        return 0.0f;
    }

    /* rpm = 60 s * 1e6 us/s / (period_us * ticks_per_rev) */
    float magnitude = 60000000.0f / ((float)period * (float)ENCODER_TICKS_PER_REV);
    return magnitude * (float)dir;
}

}

#endif // ENABLE_ENCODERS
