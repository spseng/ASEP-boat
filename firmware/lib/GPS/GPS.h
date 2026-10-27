#pragma once

#include <PiLink.h>
#include <pins.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

class GPS {
public:
    GPS(PiLink& piLink);

    void begin();

    void update();

private:
    PiLink& piLink;

    HardwareSerial GpsSerial;
    TinyGPSPlus gps;

    TinyGPSCustom ggaFixQ ;
    TinyGPSCustom ggaTime;
    TinyGPSCustom ackCmd;
    TinyGPSCustom ackFlag;

    static int32_t toE7(const RawDegrees &r) {
        int32_t v = (int32_t)r.deg * 10000000L + (int32_t)(r.billionths / 100);
        return r.negative ? -v : v;
    }
};