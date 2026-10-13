#pragma once

#include <wirelink/framing.h>
#include <wirelink/msg/common.h>

namespace wirelink::msg::lora {

    constexpr size_t MAX_LORA_PAYLOAD = 64;

    enum class MsgType : uint8_t {
        Disable = 0,

        // from-peer
        SelfStatus = 1,

        // from-land
        Command = 2,
        Heartbeat = 3,
        Reenable = 4,
        SetMode = 5,
    };

    struct Disable  {
        constexpr static MsgType TYPE = MsgType::Disable;

        uint8_t rx_id;

        template <class F>
        void fields (F& f) { f(rx_id); }
    };

    struct SelfStatus {
        constexpr static MsgType TYPE = MsgType::SelfStatus;

        wirelink::msg::common::PeerEntry self;
        float heading_deg; // degrees true, 0~360
        uint8_t state; // 0~3: mode; 4: armed; 5: gate (1=enabled); 6,7: reserved
        uint8_t faults;
        float gs_rssi;
        float gs_snr;

        template <class F>
        void fields (F& f) { f(self); f(heading_deg); f(state); f(faults); f(gs_rssi); f(gs_snr); }
    };

    struct Command {
        constexpr static MsgType TYPE = MsgType::Command;
        uint8_t rx_id;
        int16_t lin_vel; // mm/s
        int16_t ang_vel; // mrad/s

        template <class F>
        void fields (F& f) { f(rx_id); f(lin_vel); f(ang_vel); }
    };

    struct Heartbeat {
        constexpr static MsgType TYPE = MsgType::Heartbeat;

        template <class F>
        void fields (F& f) {}
    };

    struct Reenable {
        constexpr static MsgType TYPE = MsgType::Reenable;
        uint8_t rx_id;

        template <class F>
        void fields (F& f) { f(rx_id); }
    };

    struct SetMode {
        constexpr static MsgType TYPE = MsgType::SetMode;
        uint8_t rx_id;
        uint8_t mode;
        uint8_t armed;

        template <class F>
        void fields (F& f) { f(rx_id); f(mode); f(armed); }
    };
}