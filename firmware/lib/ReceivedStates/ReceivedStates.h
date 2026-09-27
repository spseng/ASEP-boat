#pragma once

#include <wirelink/wirelink.h>
#include <wirelink/msg/serial.h>

#include <optional>
#include <array>

struct ReceivedState {
    wirelink::Frame frame;
    uint32_t timestamp;
};

class ReceivedStates {
public:
    ReceivedStates();

    bool cache(const wirelink::Frame& frame, uint32_t timestamp);

    std::optional<uint32_t> timestamp(wirelink::msg::serial::MsgType type) const;

    template<class T>
    std::optional<T> poll(wirelink::msg::serial::MsgType type) const {
        size_t index = index_of(type).value_or(-1);
        if (index == -1) { return std::nullopt; }
        T out;
        if (!wirelink::codec::unpack(states[index].frame, out)) { return std::nullopt; }
        return out;
    }
private:
    constexpr static wirelink::msg::serial::MsgType receive_types[] = {
        wirelink::msg::serial::MsgType::MotorCommand,
        wirelink::msg::serial::MsgType::BroadcastPayload,
        wirelink::msg::serial::MsgType::Config,
        wirelink::msg::serial::MsgType::Heartbeat,
    };
    std::optional<size_t> index_of(wirelink::msg::serial::MsgType type) const;

    std::array<ReceivedState, sizeof(receive_types) / sizeof(receive_types[0])> states;
};