#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>

template <class T, size_t N>
struct List {
    std::array<T, N> items;
    uint8_t count = 0;
};

enum class MsgType : uint8_t {
    //upstream
    Attitude = 1,
    GPS   = 2,
    PeerTable = 3,
    OwnScalar = 4,
    ReceivedCommand = 5,
    Status = 6,

    //downstream
    MotorCommand = 7,
    BroadcastPayload = 8,
    Config = 9,
    Heartbeat = 10,
};

namespace protocol {
    // 0x00(1) + COBS(1) + body(1+1+1+248+2) + 0x00(1) = 256
    // body = type(1) + seq(1) + len(1) + payload(248) + crc(2)

    constexpr uint8_t START_BYTE = 0x00;
    constexpr uint8_t END_BYTE = 0x00;
    constexpr size_t MAX_PAYLOAD = 248;
    constexpr size_t MAX_BODY = MAX_PAYLOAD + 5;
    constexpr size_t MAX_ENCODED = MAX_BODY + 1;
    constexpr size_t MAX_BUFFER = MAX_ENCODED + 2;

    constexpr size_t MAX_PEER = 10;

    struct Frame {
        uint8_t type;
        uint8_t seq;
        uint8_t len;
        std::array<uint8_t, MAX_PAYLOAD> payload;
        uint16_t crc;
    };
    
    struct Attitude {
        constexpr static MsgType TYPE = MsgType::Attitude;
        float quat_x;
        float quat_y;
        float quat_z;
        float quat_w;
        float ang_vel_x;
        float ang_vel_y;
        float ang_vel_z;
        float lin_acc_x;
        float lin_acc_y;
        float lin_acc_z;
        uint8_t calib_status;

        template <class F>
        void fields (F& f) {
            f(quat_x); f(quat_y); f(quat_z); f(quat_w);
            f(ang_vel_x); f(ang_vel_y); f(ang_vel_z);
            f(lin_acc_x); f(lin_acc_y); f(lin_acc_z);
            f(calib_status);
        }
    };

    struct GPS {
        constexpr static MsgType TYPE = MsgType::GPS;
        int32_t lat;
        int32_t lon;
        uint64_t utc_time_ms;
        uint8_t fix_quality;
        uint8_t satellites;
        float hdop;
        float speed_over_ground;
        float course_over_ground;

        template <class F>
        void fields (F& f) {
            f(lat); f(lon); f(utc_time_ms);
            f(fix_quality); f(satellites);
            f(hdop); f(speed_over_ground); f(course_over_ground);
        }
    };

    struct PeerTable {
        constexpr static MsgType TYPE = MsgType::PeerTable;
        
        struct PeerEntry {
            uint8_t id;
            int32_t lat;
            int32_t lon;
            float scalar;
            uint32_t age_ms;

            template <class F>
            void fields (F& f) { f(id); f(lat); f(lon); f(scalar); f(age_ms); }
        };

        List<PeerEntry, MAX_PEER> peer_entries;

        template <class F>
        void fields (F& f) { f(peer_entries); }
    };

    struct OwnScalar {
        constexpr static MsgType TYPE = MsgType::OwnScalar;
        float scalar;

        template <class F>
        void fields (F& f) { f(scalar); }
    };

    struct ReceivedCommand {
        constexpr static MsgType TYPE = MsgType::ReceivedCommand;
        //TODO Implement
    };

    struct Status {
        constexpr static MsgType TYPE = MsgType::Status;
        uint8_t boat_id;
        uint8_t gate_state;
        uint16_t fault_flags;
        
        struct LinkStatus {
            uint8_t id;
            float rssi;
            float snr;
            uint32_t lost_packets;

            template <class F>
            void fields (F& f) { f(id); f(rssi); f(snr); f(lost_packets); }
        };

        List<LinkStatus, MAX_PEER> links;

        template <class F>
        void fields (F& f) { f(boat_id); f(gate_state); f(fault_flags); f(links); }
    };

    struct MotorCommand {
        constexpr static MsgType TYPE = MsgType::MotorCommand;
        uint16_t pwm_port;
        uint16_t pwm_starboard;

        template <class F>
        void fields (F& f) { f(pwm_port); f(pwm_starboard); }
    };

    struct BroadcastPayload {
        constexpr static MsgType TYPE = MsgType::BroadcastPayload;
        int32_t lat;
        int32_t lon;
        float scalar;

        template <class F>
        void fields (F& f) { f(lat); f(lon); f(scalar); }
    };

    struct config {
        constexpr static MsgType TYPE = MsgType::Config;
        uint8_t boat_id;
        uint8_t tdma_slot;
        uint8_t lora_sf;
        uint32_t lora_bw_hz;
        uint32_t lora_freq_hz;
        uint16_t pwm_min;
        uint16_t pwm_neutral;
        uint16_t pwm_max;
        uint8_t output_enable;

        template <class F>
        void fields (F& f) { f(boat_id); f(tdma_slot); f(lora_sf); f(lora_bw_hz); f(lora_freq_hz); f(pwm_min); f(pwm_neutral); f(pwm_max); f(output_enable); }
    };

    struct Heartbeat {
        constexpr static MsgType TYPE = MsgType::Heartbeat;
        uint8_t ros_mode;

        template <class F>
        void fields (F& f) { f(ros_mode); }
    };

    struct Reader {
        const uint8_t* buf;
        size_t len;
        size_t pos = 0;
        bool ok = true;
        
        void operator()(uint8_t& v)  { v = static_cast<uint8_t>(get(1)); }
        void operator()(uint16_t& v) { v = static_cast<uint16_t>(get(2)); }
        void operator()(uint32_t& v) { v = static_cast<uint32_t>(get(4)); }
        void operator()(uint64_t& v) { v = get(8); }
        void operator()(int32_t& v)  { v = static_cast<int32_t>(static_cast<uint32_t>(get(4))); }
        void operator()(float& v)    { uint32_t u = static_cast<uint32_t>(get(4)); std::memcpy(&v, &u, sizeof u); }
        
        template <class T, size_t N>
        void operator()(List<T, N>& list) {
            (*this)(list.count);
            if (list.count > N) { ok = false; list.count = 0; return; }
            for (size_t i = 0; i < list.count; ++i) list.items[i].fields(*this);
        }
    
        template <class T> void operator()(T&) = delete;
    
    private:
        uint64_t get(size_t n) {
            if (!ok || pos + n > len) { ok = false; return 0; }
            uint64_t v = 0;
            for (size_t i = 0; i < n; ++i) v |= static_cast<uint64_t>(buf[pos++]) << (8 * i);
            return v;
        }
    };
    
    struct Writer {
        uint8_t* buf;
        size_t cap;
        size_t pos = 0;
        bool ok = true;
    
        void operator()(uint8_t v)  { put(v, 1); }
        void operator()(uint16_t v) { put(v, 2); }
        void operator()(uint32_t v) { put(v, 4); }
        void operator()(uint64_t v) { put(v, 8); }
        void operator()(int32_t v)  { put(static_cast<uint32_t>(v), 4); }
        void operator()(float v)    { uint32_t u; std::memcpy(&u, &v, sizeof u); put(u, 4); }
    
        template <class T, size_t N>
        void operator()(List<T, N>& list) {
            if (list.count > N) { ok = false; return; }
            (*this)(list.count);
            for (size_t i = 0; i < list.count; ++i) list.items[i].fields(*this);
        }
    
        template <class T> void operator()(T) = delete;
    
    private:
        void put(uint64_t v, size_t n) {
            if (!ok || pos + n > cap) { ok = false; return; }
            for (size_t i = 0; i < n; ++i) buf[pos++] = static_cast<uint8_t>(v >> (8 * i));
        }
    };
    
    template <class T>
    bool decode(const Frame& frame, T& out) {
        if (frame.type != static_cast<uint8_t>(T::TYPE)) return false;
        Reader r{frame.payload.data(), frame.len};
        out.fields(r);
        return r.ok && r.pos == frame.len;
    }
    
    template <class T>
    bool encode(T msg, Frame& frame) {
        Writer w{frame.payload.data(), frame.payload.size()};
        msg.fields(w);
        if (!w.ok) return false;
        frame.type = static_cast<uint8_t>(T::TYPE);
        frame.len = static_cast<uint8_t>(w.pos);
        return true;
    }
}