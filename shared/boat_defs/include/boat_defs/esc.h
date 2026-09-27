#pragma once

#include <cstdint>

namespace boat::esc {
    constexpr uint16_t PWM_MIN_US = 1000; //us
    constexpr uint16_t PWM_MAX_US = 2000; //us
    constexpr uint16_t PWM_NEUTRAL_US = 1500; //us
}