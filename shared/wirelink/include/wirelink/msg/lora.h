#pragma once

#include <wirelink/framing.h>
#include <wirelink/msg/common.h>

namespace wirelink::msg::lora {

    constexpr size_t MAX_LORA_PAYLOAD = 64;

    enum class MsgType : uint8_t {
        Disable = 0,

        // peer-to-peer
        SelfStatus = 1,

        // land-to-peer
        Command = 2,

        // peer-to-land
        Status = 3,
    };

    struct SelfStatus {
        constexpr static MsgType TYPE = MsgType::SelfStatus;
        wirelink::msg::common::PeerEntry self;

        template <class F>
        void fields (F& f) { f(self); }
    };

    struct Command {
        constexpr static MsgType TYPE = MsgType::Command;
        uint8_t rx_id;
        int16_t lin_vel; // m/s
        int16_t ang_vel; // rad/s
        uint32_t age_ms;

        template <class F>
        void fields (F& f) { f(rx_id); f(lin_vel); f(ang_vel); f(age_ms); }
    };

    struct Status {
        constexpr static MsgType TYPE = MsgType::Status;
        uint8_t tx_id;
        float scalar;
        wirelink::msg::common::LinkStatus links;
        uint32_t age_ms;

        template <class F>
        void fields (F& f) { f(tx_id); f(scalar); f(links); f(age_ms); }
    };
}