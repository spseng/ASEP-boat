#include "ReceivedStates.h"

#include <algorithm>

ReceivedStates::ReceivedStates() {
    for (auto& state : states) {
        state.frame = {};
        state.timestamp = 0;
    }
}

bool ReceivedStates::cache(const wirelink::Frame& frame, uint32_t timestamp) {
    size_t index = index_of(static_cast<wirelink::msg::serial::MsgType>(frame.type)).value_or(-1);
    if (index == -1) { return false; }
    states[index].frame = frame;
    states[index].timestamp = timestamp;
    return true;
}

std::optional<uint32_t> ReceivedStates::timestamp(wirelink::msg::serial::MsgType type) const {
    size_t index = index_of(type).value_or(-1);
    if (index == -1) { return std::nullopt; }
    return states[index].timestamp;
}

std::optional<size_t> ReceivedStates::index_of(wirelink::msg::serial::MsgType type) const {
    auto it = std::find(std::begin(receive_types), std::end(receive_types), type);
    if (it != std::end(receive_types)) {
        return std::distance(std::begin(receive_types), it);
    }
    return std::nullopt;
}