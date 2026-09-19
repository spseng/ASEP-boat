#pragma once

#include <wirelink/framing.h>

namespace wirelink::msg::serial {

    constexpr size_t MAX_PEER = 10;

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

        wirelink::List<PeerEntry, MAX_PEER> peer_entries;

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

        wirelink::List<LinkStatus, MAX_PEER> links;

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

    struct Config {
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
}