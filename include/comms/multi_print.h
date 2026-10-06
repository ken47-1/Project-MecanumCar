/* ==================== multi_print.h ==================== */
#pragma once

/* =============== INCLUDES =============== */
/* ============ CORE ============ */
#include <Arduino.h>

/* =============== TYPES =============== */
/* ============ CLASSES ============ */
class MultiPrint final : public Print {
public:
    MultiPrint(Print* a, Print* b = nullptr);

    void set_secondary(Print* b);

    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buffer, size_t size) override;

private:
    Print* _a;
    Print* _b;
};