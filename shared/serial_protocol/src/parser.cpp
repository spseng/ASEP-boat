#include <serial_protocol/parser.h>

Parser::Parser() : buffer_index(0), state(readState::WAIT) {
    clear_buffer();
}

std::optional<protocol::Frame> Parser::feed(uint8_t b) {
    if (state == readState::WAIT) {
        if (b == protocol::START_BYTE) {
            Parser::clear_buffer();
            buffer[0] = protocol::START_BYTE;
            buffer_index++;
            state = readState::READ;
        }
    } else if (state == readState::READ) {
        if (b == protocol::END_BYTE) {
            if (buffer_index == 1) return std::nullopt;
            buffer[buffer_index++] = protocol::END_BYTE;
            state = readState::WAIT;
            return Parser::buffer_to_frame(buffer, buffer_index);
        }
        if (buffer_index >= protocol::MAX_BUFFER - 1) {
            state = readState::WAIT;
            return std::nullopt;
        }
        buffer[buffer_index++] = b;
    }

    return std::nullopt;
}

std::optional<size_t> Parser::frame_to_buffer(protocol::Frame& frame, std::array<uint8_t, protocol::MAX_BUFFER>& out) {
    if (frame.len > protocol::MAX_PAYLOAD) {
        return std::nullopt;
    }
    frame.seq = tx_seq[frame.type]++;
    std::array<uint8_t, protocol::MAX_BODY> body;
    body[0] = frame.type;
    body[1] = frame.seq;
    body[2] = frame.len;
    std::copy(frame.payload.begin(), frame.payload.begin() + frame.len, body.begin() + 3);
    frame.crc = crc16(body, 3 + frame.len);
    body[3 + frame.len] = static_cast<uint8_t>(frame.crc & 0xFF);
    body[4 + frame.len] = static_cast<uint8_t>(frame.crc >> 8);
    std::array<uint8_t, protocol::MAX_ENCODED> encoded;
    std::optional<size_t> encoded_len = COBS_encode(body, frame.len + 5u, encoded);
    if (!encoded_len.has_value()) {
        return std::nullopt;
    }
    out[0] = protocol::START_BYTE;
    std::copy(encoded.begin(), encoded.begin() + *encoded_len, out.begin() + 1);
    out[1 + *encoded_len] = protocol::END_BYTE;
    return *encoded_len + 2;
}

void Parser::clear_buffer() {
    buffer = std::array<uint8_t, protocol::MAX_BUFFER>();
    buffer_index = 0;
}

std::optional<protocol::Frame> Parser::buffer_to_frame(const std::array<uint8_t, protocol::MAX_BUFFER>& buffer, size_t last_index) {
    std::array<uint8_t, protocol::MAX_ENCODED> encoded;
    std::array<uint8_t, protocol::MAX_BODY> decoded;
    std::copy(buffer.begin() + 1, buffer.begin() + (last_index - 1), encoded.begin());
    std::optional<size_t> decoded_len = COBS_decode(encoded, last_index - 2, decoded);
    if (!decoded_len.has_value() || *decoded_len < 5) {
        return std::nullopt; // Decoding failed
    }
    protocol::Frame frame;
    frame.type = decoded[0];
    frame.seq = decoded[1];
    frame.len = decoded[2];
    if (*decoded_len != frame.len + 5u) {
        return std::nullopt; // Invalid length
    }
    std::copy(decoded.begin() + 3, decoded.begin() + 3 + frame.len, frame.payload.begin());
    frame.crc = decoded[3 + frame.len] | (decoded[4 + frame.len] << 8);
    if (frame.crc != crc16(decoded, 3 + frame.len)) {
        return std::nullopt; // CRC mismatch
    }
    return frame;
}

std::optional<size_t> Parser::COBS_encode(const std::array<uint8_t, protocol::MAX_BODY>& input, size_t input_len, std::array<uint8_t, protocol::MAX_ENCODED>& output) {
    if (input_len == 0 || input_len > protocol::MAX_BODY) {
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

std::optional<size_t> Parser::COBS_decode(const std::array<uint8_t, protocol::MAX_ENCODED>& input, size_t input_len, std::array<uint8_t, protocol::MAX_BODY>& output) {
    if (input_len == 0 || input_len > protocol::MAX_ENCODED) {
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

uint16_t Parser::crc16(const std::array<uint8_t, protocol::MAX_BODY>& data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i] << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}