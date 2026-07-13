/* ==================== MultiPrint.h ==================== */
#pragma once
#include <Arduino.h>

/* =============== TYPES =============== */
/* ============ CLASSES ============ */
class MultiPrint final : public Print {
public:
    MultiPrint(Print* a, Print* b = nullptr) : _a(a), _b(b) {}
    void set_secondary(Print* b) { _b = b; }

    size_t write(uint8_t c) override {
        size_t n = 0;
        if (_a) n += _a->write(c);
        if (_b) n += _b->write(c);
        return n;
    }
private:
    Print* _a;
    Print* _b;
};