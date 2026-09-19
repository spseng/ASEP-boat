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

    namespace bytes {
        struct Payload { std::array<uint8_t, MAX_PAYLOAD> data{}; size_t len = 0; };  // only payload
        struct Body    { std::array<uint8_t, MAX_BODY>    data{}; size_t len = 0; };  // type to crc
        struct Encoded { std::array<uint8_t, MAX_ENCODED> data{}; size_t len = 0; };  // COBS encoded
        struct Packet  { std::array<uint8_t, MAX_BUFFER>  data{}; size_t len = 0; };  // 0x00 to 0x00
    }

    struct Frame {
        uint8_t type;
        uint8_t seq;
        uint8_t len;
        wirelink::bytes::Payload payload;
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
    bool encode(const wirelink::bytes::Body& input, wirelink::bytes::Encoded& output);

    bool decode(const wirelink::bytes::Encoded& input, wirelink::bytes::Body& output);
}

namespace wirelink::crc {
    uint16_t crc16(const wirelink::bytes::Body& data, size_t len);
}

namespace wirelink::framing {
    std::optional<wirelink::bytes::Packet> wrap(const wirelink::Frame& frame, uint8_t seq);

    std::optional<wirelink::Frame>  unwrap(const wirelink::bytes::Packet& in);
}

namespace wirelink {
    class Link {
    public:
        Link();

        std::optional<wirelink::Frame> feed(uint8_t b);

        std::optional<wirelink::bytes::Packet> send(const wirelink::Frame& frame);
    private:
        enum class ReadState {
            WAIT,
            READ
        };
        wirelink::bytes::Packet buffer;
        ReadState state;
        std::array<uint8_t, 256> tx_seq{};
        void clear_buffer();
    };
}