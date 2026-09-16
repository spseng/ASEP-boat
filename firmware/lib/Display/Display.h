#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "pins.h"

class Display {
public:
    Display();
    void begin();
    void clear();
    void print(const char* message);
    void setCursor(int col, int row);
    void setBrightness(int level);
    
private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
};