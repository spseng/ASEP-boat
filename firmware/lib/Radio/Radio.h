#pragma once
#include <RadioLib.h>
#include <config.h>
#include <pins.h>

class Radio {
public:
    Radio();

private:
    SX1262 sx1262 = new Module(pins::RADIO_CS, pins::RADIO_DIO1, pins::RADIO_RST, pins::RADIO_BUSY);
};