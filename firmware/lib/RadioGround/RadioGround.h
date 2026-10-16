#pragma once

#include <boat_defs/boat_defs.h>
#include <wirelink/wirelink.h>

#include <GroundLink.h>
#include <RadioLib.h>
#include <config.h>
#include <pins.h>

class Radio {
public:
    Radio(GroundLink& groundLink);

    void begin();

    void update();

private:
    GroundLink& groundLink;

    SX1262 sx1262 = new Module(pins::RADIO_CS, pins::RADIO_DIO1, pins::RADIO_RST, pins::RADIO_BUSY);

    static void onDio1();
    static volatile bool rx_flag;
    bool tx_flag;
    bool may_transmit;

    uint32_t last_receive_us;
    uint8_t last_receive_tx_id;
    uint32_t my_next_transmit_us;
    uint8_t my_tx_id;

    void receive();

    void schedule_next_transmit();

    void transmit();

    uint8_t seq;
};