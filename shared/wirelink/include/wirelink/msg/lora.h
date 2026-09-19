#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::lora {

    enum class MsgType : uint8_t {
        Config = 1,
        Heartbeat = 2,
        Command = 3,
        Status = 4,
        BroadcastPayload = 5,
    };
}