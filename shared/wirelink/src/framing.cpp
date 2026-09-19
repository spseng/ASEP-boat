#include <wirelink/framing.h>

#include <algorithm>

namespace wirelink::cobs {
    std::optional<size_t> encode(const std::array<uint8_t, wirelink::MAX_BODY>& input, size_t input_len, std::array<uint8_t, wirelink::MAX_ENCODED>& output) {
        if (input_len == 0 || input_len > wirelink::MAX_BODY) {
            return std::nullopt; // Invalid input length
        }

        size_t read_index = 0;
        size_t write_index = 1;
        size_t code_index = 0;
        uint8_t code = 1;

        while (read_index < input_len) {
            if (input[read_index] == 0) {
                output[code_index] = code;
                code_index = write_index++;
                code = 1; // Reset code
            } else {
                output[write_index++] = input[read_index];
                ++code;
                if (code == 0xFF) { // If code reaches 255, we need to start a new block
                    output[code_index] = code;
                    code_index = write_index++;
                    code = 1; // Reset code
                }
            }
            ++read_index;
        }

        output[code_index] = code;

        return write_index; // Return the number of bytes written to output
    }

    std::optional<size_t> decode(const std::array<uint8_t, wirelink::MAX_ENCODED>& input, size_t input_len, std::array<uint8_t, wirelink::MAX_BODY>& output) {
        if (input_len == 0 || input_len > wirelink::MAX_ENCODED) {
            return std::nullopt; // Invalid input length
        }

        size_t read_index = 0;
        size_t write_index = 0;

        while (read_index < input_len) {
            uint8_t code = input[read_index++];
            if (code == 0) {
                return std::nullopt; // Invalid COBS encoding
            }

            for (uint8_t i = 1; i < code; ++i) {
                if (read_index >= input_len) {
                    return std::nullopt; // Not enough data
                }
                output[write_index++] = input[read_index++];
            }

            if (code < 0xFF && read_index < input_len) {
                output[write_index++] = 0; // Insert zero byte
            }
        }

        return write_index; // Return the number of bytes written to output
    }   
}

uint16_t wirelink::crc::crc16(const std::array<uint8_t, wirelink::MAX_BODY>& data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i] << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

namespace wirelink::framing {
    std::optional<size_t> wrap(const wirelink::Frame& frame, uint8_t seq, std::array<uint8_t, wirelink::MAX_BUFFER>& out) {
        if (frame.len > wirelink::MAX_PAYLOAD) {
            return std::nullopt;
        }

        std::array<uint8_t, MAX_BODY> body;
        body[0] = frame.type;
        body[1] = seq;
        body[2] = frame.len;
        std::copy(frame.payload.begin(), frame.payload.begin() + frame.len, body.begin() + 3);

        const uint16_t crc = crc::crc16(body, 3 + frame.len);
        body[3 + frame.len] = static_cast<uint8_t>(crc & 0xFF);
        body[4 + frame.len] = static_cast<uint8_t>(crc >> 8);
        std::array<uint8_t, wirelink::MAX_ENCODED> encoded;
        std::optional<size_t> encoded_len = wirelink::cobs::encode(body, frame.len + 5u, encoded);
        if (!encoded_len.has_value()) {
            return std::nullopt;
        }
        out[0] = wirelink::START_BYTE;
        std::copy(encoded.begin(), encoded.begin() + *encoded_len, out.begin() + 1);
        out[1 + *encoded_len] = wirelink::END_BYTE;
        return *encoded_len + 2;
    }

    std::optional<wirelink::Frame> unwrap(const std::array<uint8_t, wirelink::MAX_BUFFER>& in, size_t total_len) {
        if (total_len < 8 || total_len > wirelink::MAX_BUFFER) {
            return std::nullopt; // Invalid total length
        }
        std::array<uint8_t, wirelink::MAX_ENCODED> encoded;
        std::array<uint8_t, wirelink::MAX_BODY> decoded;
        std::copy(in.begin() + 1, in.begin() + (total_len - 1), encoded.begin());
        std::optional<size_t> decoded_len = wirelink::cobs::decode(encoded, total_len - 2, decoded);
        if (!decoded_len.has_value() || *decoded_len < 5) {
            return std::nullopt; // Decoding failed
        }
        wirelink::Frame frame;
        frame.type = decoded[0];
        frame.seq = decoded[1];
        frame.len = decoded[2];
        if (*decoded_len != frame.len + 5u) {
            return std::nullopt; // Invalid length
        }
        std::copy(decoded.begin() + 3, decoded.begin() + 3 + frame.len, frame.payload.begin());
        frame.crc = decoded[3 + frame.len] | (decoded[4 + frame.len] << 8);
        if (frame.crc != wirelink::crc::crc16(decoded, 3 + frame.len)) {
            return std::nullopt; // CRC mismatch
        }
        return frame;
    }
}

namespace wirelink {
    Link::Link() : buffer_index(0), state(ReadState::WAIT) {
        clear_buffer();
    }

    std::optional<wirelink::Frame> Link::feed(uint8_t b) {
        if (state == Link::ReadState::WAIT) {
            if (b == wirelink::START_BYTE) {
                Link::clear_buffer();
                buffer[0] = wirelink::START_BYTE;
                buffer_index ++;
                state = ReadState::READ;
            }
            return std::nullopt;
        }

        if (b == wirelink::END_BYTE) {
            if (buffer_index == 1) return std::nullopt;
            buffer[buffer_index++] = wirelink::END_BYTE;
            state = ReadState::WAIT;
            return wirelink::framing::unwrap(buffer, buffer_index); // pass length
        }

        if (buffer_index >= wirelink::MAX_BUFFER - 1) {
            state = ReadState::WAIT;
            return std::nullopt;
        }
        buffer[buffer_index++] = b;

        return std::nullopt;
    }

    std::optional<size_t> Link::send(const wirelink::Frame& frame, std::array<uint8_t, wirelink::MAX_BUFFER>& out) {
        const uint8_t seq = tx_seq[frame.type]++;
        return wirelink::framing::wrap(frame, seq, out);
    }

    void Link::clear_buffer() {
        buffer.fill(0);
        buffer_index = 0;
    }
}