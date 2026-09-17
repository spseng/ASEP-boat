#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <iterator>
#include <algorithm>
#include <vector>
#include <serial_protocol/protocol.h>

enum readState {
    WAIT,
    READ
};

class Parser {
public:
    Parser();

    std::optional<protocol::Frame> feed(uint8_t b);

    std::optional<size_t> frame_to_buffer(protocol::Frame& frame, std::array<uint8_t, protocol::MAX_BUFFER>& out);
private:
    std::array<uint8_t, protocol::MAX_BUFFER> buffer;

    size_t buffer_index;

    readState state;

    std::array<uint8_t, 256> tx_seq{};

    void clear_buffer();

    std::optional<protocol::Frame> buffer_to_frame(const std::array<uint8_t, protocol::MAX_BUFFER>& bytes, size_t last_index);

    static std::optional<size_t> COBS_encode(const std::array<uint8_t, protocol::MAX_BODY>& input, size_t input_len, std::array<uint8_t, protocol::MAX_ENCODED>& output);

    static std::optional<size_t> COBS_decode(const std::array<uint8_t, protocol::MAX_ENCODED>& input, size_t input_len, std::array<uint8_t, protocol::MAX_BODY>& output);

    static uint16_t crc16(const std::array<uint8_t, protocol::MAX_BODY>& data, size_t len);
};