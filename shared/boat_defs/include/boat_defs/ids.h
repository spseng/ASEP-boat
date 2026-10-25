#pragma once

#include <cstdint>

namespace boat::ids {
    constexpr uint8_t GROUND_STATION_ID = 0;
    constexpr uint8_t BOAT_ID_MIN = 1;
    constexpr uint8_t BOAT_ID_MAX = 3;
    constexpr uint8_t NUM_IDS = BOAT_ID_MAX + 1;
    constexpr uint8_t BROADCAST_ID = 255;
    constexpr bool is_valid_boat_id(uint8_t id) {
        return id >= BOAT_ID_MIN && id <= BOAT_ID_MAX;
    }
    constexpr bool is_valid_lora_id(uint8_t id) {
        return is_valid_boat_id(id) || id == GROUND_STATION_ID || id == BROADCAST_ID;
    }
    constexpr uint8_t MAX_BOAT_NUM = BOAT_ID_MAX - BOAT_ID_MIN + 1;
}