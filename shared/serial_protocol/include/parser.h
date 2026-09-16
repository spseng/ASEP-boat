#include <cstddef>
#include <cstdint>
#include <array>
#include "types.h"

class Parser {
public:
    Parser();

    bool feed(uint8_t b);
private:
    std::array<uint8_t, protocol::MAX_BUFFER> buffer;

    size_t buffer_index;

    std::array<uint8_t, protocol::MAX_BUFFER> COBS_decode(const std::array<uint8_t, protocol::MAX_BUFFER>& input, size_t length);
};