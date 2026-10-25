#pragma once

#include <wirelink/framing.h>
#include <wirelink/msg/common.h>

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
        BroadcastEntry = 8,
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

        wirelink::List<wirelink::msg::common::PeerEntry, MAX_PEER> peer_entries;

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
        
        uint8_t lora_type;
        wirelink::bytes::Payload payload;

        template <class F>
        void fields (F& f) { f(lora_type); f(payload); }
    };

    struct Status {
        constexpr static MsgType TYPE = MsgType::Status;
        uint8_t boat_id;
        uint8_t gate_state;
        uint16_t fault_flags;

        wirelink::List<wirelink::msg::common::LinkStatus, MAX_PEER> links;

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

    struct BroadcastEntry {
        constexpr static MsgType TYPE = MsgType::BroadcastEntry;
        wirelink::msg::common::PeerEntry self;

        template <class F>
        void fields (F& f) { f(self); }
    };

    struct Config {
        constexpr static MsgType TYPE = MsgType::Config;
        uint8_t boat_id;
        uint8_t gate_enable;

        template <class F>
            void fields (F& f) { f(boat_id); f(gate_enable); }
        };

    struct Heartbeat {
        constexpr static MsgType TYPE = MsgType::Heartbeat;
        uint8_t ros_mode;
        uint8_t ros_armed;

        template <class F>
        void fields (F& f) { f(ros_mode); }
    };
}