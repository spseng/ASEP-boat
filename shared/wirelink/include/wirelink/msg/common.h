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

/*
struct SelfStatus {
    constexpr static MsgType TYPE = MsgType::SelfStatus;

    uint8_t state;     // 1 B (0~3: mode; 4: armed; 5: gate; 6,7: reserved)
    uint8_t faults;    // 1 B
    float gs_rssi;     // 4 B
    float gs_snr;      // 4 B

    uint8_t id;        // 1 B
    int32_t lat;       // 4 B
    int32_t lon;       // 4 B
    float scalar;      // 4 B
    float heading_deg; // 4 B (degrees true, 0~360)
};

*/