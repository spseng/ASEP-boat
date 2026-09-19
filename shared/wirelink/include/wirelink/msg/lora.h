#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::lora {

    enum class MsgType : uint8_t {
        // peer-to-peer
        SelfStatus = 1,

        // land-to-peer
        Command = 2,

        // peer-to-land
        Status = 3,
    };
}