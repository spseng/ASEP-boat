#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::common {
    struct PeerEntry {
        uint8_t id;
        int32_t lat;
        int32_t lon;
        float scalar;
        uint32_t age_ms;
        template <class F>
        void fields (F& f) { f(id); f(lat); f(lon); f(scalar); f(age_ms); }
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