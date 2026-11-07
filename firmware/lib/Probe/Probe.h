#pragma once

#include <PiLink.h>
#include <OneWire.h>
#include <config.h>
#include <pins.h>

class Probe {
public:
    Probe(PiLink& piLink);

    void begin();
    void update();

private:
    PiLink& piLink;

    float tempC;

    OneWire oneWire{pins::DS18B20};
    
    bool converting = false;
    uint32_t lastStart = 0;

    bool startConversion();
    bool readTemp(float& temp);
};
