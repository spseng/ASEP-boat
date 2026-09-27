#pragma once

#include <cstdint>

namespace boat::esc {
    constexpr uint16_t PWM_MIN = 1000; //us
    constexpr uint16_t PWM_MAX = 2000; //us
    constexpr uint16_t PWM_NEUTRAL = 1500; //us
}