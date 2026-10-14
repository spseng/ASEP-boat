#pragma once

#include <wirelink/framing.h>
#include <wirelink/msg/common.h>

// Local to the PC <-> base-station ESP32 serial link; never sent over LoRa.
// Types start at 0x80 so they never collide with a lora::MsgType.
// All optional: the PC app works without them.

// RxInfo: sent immediately after each relayed LoRa frame, describing how
// well the base station received that frame.

namespace wirelink::msg::base {

    constexpr uint8_t FIRST_LOCAL_TYPE = 0x80;

    enum class MsgType : uint8_t {
        RxInfo     = 0x80,
        BaseStatus = 0x81,
        BasePosition = 0x82,
    };

    constexpr bool is_local_type(uint8_t type) { return type >= FIRST_LOCAL_TYPE; }

    struct RxInfo {
        constexpr static MsgType TYPE = MsgType::RxInfo;
        float rssi; // dBm
        float snr; // dB

        template <class F>
        void fields (F& f) { f(rssi); f(snr); }
    };

    struct BaseStatus {
        constexpr static MsgType TYPE = MsgType::BaseStatus;
        uint32_t uptime_ms;
        uint32_t rx_ok;      // LoRa packets received and relayed
        uint32_t rx_bad;     // LoRa packets dropped (CRC error / too long)
        uint32_t tx_count;   // frames transmitted over LoRa

        template <class F>
        void fields(F& f) { f(uptime_ms); f(rx_ok); f(rx_bad); f(tx_count); }
    };

    struct BasePosition {
        constexpr static MsgType TYPE = MsgType::BasePosition;
        int32_t lat;          // 1e-7 deg
        int32_t lon;          // 1e-7 deg
        uint8_t fix_quality;  // 0 = no fix (NMEA GGA quality otherwise)
        uint8_t satellites;

        template <class F>
        void fields(F& f) { f(lat); f(lon); f(fix_quality); f(satellites); }
    };

}