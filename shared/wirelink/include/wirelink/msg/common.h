#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::common {
    struct PeerEntry {
        uint8_t id;
        int32_t lat;
        int32_t lon;
        float scalar;
        float heading_deg; // degrees true, 0~360

        template <class F>
        void fields (F& f) { f(id); f(lat); f(lon); f(scalar); f(heading_deg); }
    };

    struct LinkStatus {
        uint8_t id;
        float rssi;
        float snr;
        uint32_t age_ms;

        template <class F>
        void fields (F& f) { f(id); f(rssi); f(snr); f(age_ms); }
    };
}