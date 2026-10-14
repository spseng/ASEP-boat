#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::common {
    struct PeerEntry {
        uint8_t id;
        int32_t lat;
        int32_t lon;
        float scalar;

        template <class F>
        void fields (F& f) { f(id); f(lat); f(lon); f(scalar); }
    };

    struct LinkStatus {
        uint8_t id;
        float rssi;
        float snr;
        uint32_t lost_packets;

        template <class F>
        void fields (F& f) { f(id); f(rssi); f(snr); f(lost_packets); }
    };
}