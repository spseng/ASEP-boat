#pragma once

#include <PiLink.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class IMU {
public:
    IMU(PiLink& piLink);
    void begin();
    void update();
private:
    PiLink& piLink;
    
    Adafruit_BNO055 bno;

    bool ok_ = true;
    unsigned long last_ms = 0;
};
