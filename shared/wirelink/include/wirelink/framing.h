#pragma once

#include <optional>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>

namespace wirelink {

    constexpr uint8_t START_BYTE = 0x00;
    constexpr uint8_t END_BYTE = 0x00;
    constexpr size_t MAX_PAYLOAD = 248;
    constexpr size_t MAX_BODY = MAX_PAYLOAD + 5;
    constexpr size_t MAX_ENCODED = MAX_BODY + 1;
    constexpr size_t MAX_BUFFER = MAX_ENCODED + 2;

    struct Frame {
        uint8_t type;
        uint8_t seq;
        uint8_t len;
        std::array<uint8_t, MAX_PAYLOAD> payload;
        uint16_t crc;
    };
}

namespace wirelink {
    template <class T, size_t N>
    struct List {
        std::array<T, N> items;
        uint8_t count = 0;
    };
}

namespace wirelink::cobs {
    std::optional<size_t> encode(const std::array<uint8_t, wirelink::MAX_BODY>& input, size_t input_len, std::array<uint8_t, wirelink::MAX_ENCODED>& output);

    std::optional<size_t> decode(const std::array<uint8_t, wirelink::MAX_ENCODED>& input, size_t input_len, std::array<uint8_t, wirelink::MAX_BODY>& output);
}

namespace wirelink::crc {
    uint16_t crc16(const std::array<uint8_t, wirelink::MAX_BODY>& data, size_t len);
}

namespace wirelink::framing {
    std::optional<size_t> wrap(const wirelink::Frame& frame, uint8_t seq, std::array<uint8_t, wirelink::MAX_BUFFER>& out);

    std::optional<wirelink::Frame>  unwrap(const std::array<uint8_t, wirelink::MAX_BUFFER>& in, size_t total_len);
}

namespace wirelink {
    class Link {
    public:
        Link();

        std::optional<wirelink::Frame> feed(uint8_t b);

        std::optional<size_t> send(const wirelink::Frame& frame, std::array<uint8_t, wirelink::MAX_BUFFER>& out);
    private:
        enum class ReadState {
            WAIT,
            READ
        };
        std::array<uint8_t, MAX_BUFFER> buffer;
        size_t buffer_index;
        ReadState state;
        std::array<uint8_t, 256> tx_seq{};
        void clear_buffer();
    };
}