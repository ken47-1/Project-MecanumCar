/* ==================== multi_print.cpp ==================== */
#include "comms/multi_print.h"

/* =============== PUBLIC API =============== */
MultiPrint::MultiPrint(Print* a, Print* b)
    : _a(a), _b(b) {}

void MultiPrint::set_secondary(Print* b) {
    _b = b;
}

size_t MultiPrint::write(uint8_t c) {
    bool wrote = false;

    if (_a) {
        _a->write(c);
        wrote = true;
    }
    if (_b) {
        _b->write(c);
        wrote = true;
    }

    return wrote ? 1 : 0;
}

size_t MultiPrint::write(const uint8_t* buffer, size_t size) {
    if (_a) _a->write(buffer, size);
    if (_b) _b->write(buffer, size);
    return size;
}