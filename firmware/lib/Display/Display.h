#pragma once
#include <U8g2lib.h>
#include "pins.h"

class Display {
public:
    Display();
    
private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 = U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, pins::DISPLAY_RST, pins::DISPLAY_SCL, pins::DISPLAY_SDA);
};