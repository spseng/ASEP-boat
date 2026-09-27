#pragma once

#include <cstdint>

namespace config {

    namespace esc_pwm {
        constexpr uint32_t RESOLUTION_HZ = 1000000; // 1 MHz -> 1 tick = 1 us
        constexpr uint32_t FREQUENCY = 50;
        constexpr uint32_t PERIOD_TICKS = RESOLUTION_HZ / FREQUENCY; // 20 ms frame
    }

    namespace radio {
        constexpr float TCXO_VOLTAGE = 1.8;
        constexpr float FREQUENCY = 915.0;
        constexpr float BANDWIDTH = 125.0;
        constexpr uint8_t SPREADING_FACTOR = 7;
        constexpr uint8_t POWER = 14;
    }
}