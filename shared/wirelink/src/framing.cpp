#include <wirelink/framing.h>

#include <algorithm>

namespace wirelink::cobs {
    bool encode(const wirelink::bytes::Body& input, wirelink::bytes::Encoded& output) {
        if (input.len == 0 || input.len > wirelink::MAX_BODY) {
            return false; // Invalid input length
        }

        size_t read_index = 0;
        size_t write_index = 1;
        size_t code_index = 0;
        uint8_t code = 1;

        while (read_index < input.len) {
            if (input.data[read_index] == 0) {
                output.data[code_index] = code;
                code_index = write_index++;
                code = 1; // Reset code
            } else {
                output.data[write_index++] = input.data[read_index];
                ++code;
                if (code == 0xFF) { // If code reaches 255, we need to start a new block
                    output.data[code_index] = code;
                    code_index = write_index++;
                    code = 1; // Reset code
                }
            }
            ++read_index;
        }

        output.data[code_index] = code;
        output.len = write_index;
        return true;
    }

    bool decode(const wirelink::bytes::Encoded& input, wirelink::bytes::Body& output) {
        if (input.len == 0 || input.len > wirelink::MAX_ENCODED) {
            return false; // Invalid input length
        }

        size_t read_index = 0;
        size_t write_index = 0;

        while (read_index < input.len) {
            uint8_t code = input.data[read_index++];
            if (code == 0) {
                return false; // Invalid COBS encoding
            }

            for (uint8_t i = 1; i < code; ++i) {
                if (read_index >= input.len) {
                    return false; // Not enough data
                }
                output.data[write_index++] = input.data[read_index++];
            }

            if (code < 0xFF && read_index < input.len) {
                output.data[write_index++] = 0; // Insert zero byte
            }
        }

        output.len = write_index;
        return true; 
    }   
}

uint16_t wirelink::crc::crc16(const wirelink::bytes::Body& data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data.data[i] << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

namespace wirelink::framing {

    std::optional<wirelink::bytes::Packet> wrap(const wirelink::Frame& frame, uint8_t seq) {
        if (frame.len > wirelink::MAX_PAYLOAD) {
            return std::nullopt;
        }

        wirelink::bytes::Body body;
        body.data[0] = frame.type;
        body.data[1] = seq;
        body.data[2] = frame.len;
        std::copy(frame.payload.data.begin(), frame.payload.data.begin() + frame.len, body.data.begin() + 3);

        const uint16_t crc = crc::crc16(body, 3 + frame.len);
        body.data[3 + frame.len] = static_cast<uint8_t>(crc & 0xFF);
        body.data[4 + frame.len] = static_cast<uint8_t>(crc >> 8);
        body.len = 5 + frame.len;

        wirelink::bytes::Encoded encoded;
        if (!wirelink::cobs::encode(body, encoded)) {
            return std::nullopt;
        }

        wirelink::bytes::Packet out;
        out.data[0] = wirelink::START_BYTE;
        std::copy(encoded.data.begin(), encoded.data.begin() + encoded.len, out.data.begin() + 1);
        out.data[1 + encoded.len] = wirelink::END_BYTE;
        out.len = encoded.len + 2;
        return out;
    }

    std::optional<wirelink::Frame> unwrap(const wirelink::bytes::Packet& in) {
        if (in.len < 8 || in.len > wirelink::MAX_BUFFER) {
            return std::nullopt; // too short or too long to be a frame
        }

        wirelink::bytes::Encoded encoded;
        encoded.len = in.len - 2;
        std::copy(in.data.begin() + 1, in.data.begin() + (in.len - 1), encoded.data.begin());

        wirelink::bytes::Body body;
        if (!wirelink::cobs::decode(encoded, body) || body.len < 5) {
            return std::nullopt; // corrupt COBS
        }

        wirelink::Frame frame;
        frame.type = body.data[0];
        frame.seq = body.data[1];
        frame.len = body.data[2];
        if (body.len != frame.len + 5u) {
            return std::nullopt; // len field disagrees with the decoded size
        }

        std::copy(body.data.begin() + 3, body.data.begin() + 3 + frame.len, frame.payload.data.begin());

        frame.crc = static_cast<uint16_t>(body.data[3 + frame.len] | (body.data[4 + frame.len] << 8));
        if (frame.crc != crc::crc16(body, 3 + frame.len)) {
            return std::nullopt; // CRC mismatch
        }

        return frame;
    }
}

namespace wirelink {
    Link::Link() : state(ReadState::WAIT) {
        clear_buffer();
    }

    std::optional<wirelink::Frame> Link::feed(uint8_t b) {
        if (state == Link::ReadState::WAIT) {
            if (b == wirelink::START_BYTE) {
                Link::clear_buffer();
                buffer.data[0] = wirelink::START_BYTE;
                buffer.len = 1;
                state = ReadState::READ;
            }
            return std::nullopt;
        }

        if (b == wirelink::END_BYTE) {
            if (buffer.len == 1) return std::nullopt;
            buffer.data[buffer.len++] = wirelink::END_BYTE;
            state = ReadState::WAIT;
            return wirelink::framing::unwrap(buffer); // pass length
        }

        if (buffer.len >= wirelink::MAX_BUFFER - 1) {
            state = ReadState::WAIT;
            return std::nullopt;
        }
        buffer.data[buffer.len++] = b;

        return std::nullopt;
    }

    std::optional<wirelink::bytes::Packet> Link::send(const wirelink::Frame& frame) {
        const uint8_t seq = tx_seq[frame.type]++;
        return wirelink::framing::wrap(frame, seq);
    }

    void Link::clear_buffer() {
        buffer.data.fill(0);
        buffer.len = 0;
    }
}