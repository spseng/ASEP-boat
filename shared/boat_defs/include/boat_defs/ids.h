#pragma once

#include <cstdint>

namespace boat::ids {
    constexpr uint8_t GROUND_STATION_ID = 0;
    constexpr uint8_t BOAT_ID_MIN = 1;
    constexpr uint8_t BOAT_ID_MAX = 10;
    constexpr uint8_t BROADCAST_ID = 255;
    constexpr bool is_valid_boat_id(uint8_t id) {
        return (id >= BOAT_ID_MIN && id <= BOAT_ID_MAX) || id == GROUND_STATION_ID || id == BROADCAST_ID;
    }
}