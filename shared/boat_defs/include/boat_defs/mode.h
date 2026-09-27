#pragma once

#include <cstdint>

namespace boat::mode {
    enum class Mode : uint8_t {
        MANUAL = 0,
        AUTONOMOUS = 1,
        RETURN_TO_HOME = 2,
        EMERGENCY_STOP = 3
    };

    enum class ArmedState : uint8_t {
        DISARMED = 0,
        ARMED = 1
    };

    enum class GateState : uint8_t {
        TRIPPED = 0,
        ENABLED = 1
    };

    namespace fault {
        constexpr uint16_t NONE                  = 0;
        constexpr uint16_t GPS_FAILURE           = 1 << 0;
        constexpr uint16_t COMMUNICATION_FAILURE = 1 << 1;
        constexpr uint16_t MOTOR_FAILURE         = 1 << 2;
        constexpr uint16_t BATTERY_LOW           = 1 << 3;
        constexpr uint16_t SENSOR_FAILURE        = 1 << 4;
    }
}