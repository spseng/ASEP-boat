#pragma once

namespace config {
    namespace serial {
        constexpr int BAUD_RATE = 115200;
    }

    namespace radio {
        constexpr float TCXO_VOLTAGE = 1.8;
        constexpr float FREQUENCY = 915.0;
        constexpr float BANDWIDTH = 125.0;
        constexpr int SPREADING_FACTOR = 7;
        constexpr int POWER = 14;
    }
}