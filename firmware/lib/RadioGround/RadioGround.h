#pragma once

#include <boat_defs/boat_defs.h>
#include <wirelink/wirelink.h>

#include <PiLink.h>
#include <RadioLib.h>
#include <config.h>
#include <pins.h>

class Radio {
public:
    Radio(PiLink& piLink);

    void begin(uint8_t my_tx_id);

    void update();

private:
    PiLink& piLink;

    SX1262 sx1262 = new Module(pins::RADIO_CS, pins::RADIO_DIO1, pins::RADIO_RST, pins::RADIO_BUSY);

    static IRAM_ATTR void onDio1() {rx_flag = true; };
    static volatile bool rx_flag;

    uint32_t last_receive_us;
    uint8_t last_receive_tx_id;
    uint32_t my_next_transmit_us;
    uint8_t my_tx_id;

    void schedule_next_transmit();

    void transmit();
};

volatile bool Radio::rx_flag = false;