#pragma once

#include <boat_defs/boat_defs.h>
#include <wirelink/wirelink.h>

#include <PiLink.h>
#include <ModeManager.h>
#include <RadioLib.h>
#include <config.h>
#include <pins.h>

class Radio {
public:
    Radio(PiLink& piLink, ModeManager& modeManager);

    void begin(uint8_t my_tx_id);

    void update();
    
    std::array<wirelink::msg::common::LinkStatus, boat::ids::NUM_IDS> link_statuses;
    std::array<bool, boat::ids::NUM_IDS> link_statuses_received;
    std::array<uint32_t, boat::ids::NUM_IDS> link_statuses_last_receive_ms;

private:
    PiLink& piLink;

    ModeManager& modeManager;

    SX1262 sx1262 = new Module(pins::RADIO_CS, pins::RADIO_DIO1, pins::RADIO_RST, pins::RADIO_BUSY);

    static void onDio1();
    static volatile bool rx_flag;
    bool tx_flag;
    bool may_transmit;

    uint32_t last_receive_us;
    uint8_t last_receive_tx_id;
    uint32_t my_next_transmit_us;
    uint8_t my_tx_id;

    uint8_t seq;

    void receive();

    void schedule_next_transmit();

    void transmit();

    void update_link_status(const uint8_t id, const float rssi, const float snr);

    std::array<wirelink::msg::common::PeerEntry, boat::ids::NUM_IDS> peer_map; // refer to link_statuses for validity and age of entries

    void update_peer_map(const uint8_t id, const int32_t lat, const int32_t lon, const float scalar, const float heading_deg);

    void send_peer_table();
};