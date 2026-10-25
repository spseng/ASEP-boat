#pragma once
#include <Arduino.h>

class Every {
public:
    explicit Every(uint32_t period_ms) : period_(period_ms) {}

    bool operator()() {
        const uint32_t now = millis();
        if (now - last_ < period_) return false;
        last_ = now;
        return true;
    }

private:
    uint32_t period_;
    uint32_t last_ = 0;
};