#pragma once

#include <cstdint>

namespace boat::lora {
    namespace config {
        constexpr float FREQUENCY = 915.0; // MHz
        constexpr float BANDWIDTH = 500.0; // kHz
        constexpr uint8_t SPREADING_FACTOR = 6;
        constexpr uint32_t TRANSMIT_INTERVAL = 25000; // us
    }
}