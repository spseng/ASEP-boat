#pragma once

#include <cstdint>
#include <boat_defs/boat_defs.h>

namespace config {

    namespace esc_pwm {
        constexpr uint32_t RESOLUTION_HZ = 1000000; // 1 MHz -> 1 tick = 1 us
        constexpr uint32_t FREQUENCY = 50;
        constexpr uint32_t PERIOD_TICKS = RESOLUTION_HZ / FREQUENCY; // 20 ms frame
    }

    namespace imu {
        constexpr uint8_t I2C_ADDRESS = 0x28;
        constexpr uint8_t I2C_ADDRESS_ALT = 0x29;
        constexpr uint32_t I2C_SPEED_HZ = 400000; // 400 kHz
    }

    namespace radio {
        constexpr float TCXO_VOLTAGE = 1.8;
        constexpr float FREQUENCY = boat::lora::config::FREQUENCY; // kHz
        constexpr float BANDWIDTH = boat::lora::config::BANDWIDTH;
        constexpr uint8_t SPREADING_FACTOR = boat::lora::config::SPREADING_FACTOR;
        constexpr uint8_t POWER = 22; // dBm
    }

    namespace ground_link {
        constexpr uint32_t BAUD_RATE = 115200;
    }
}