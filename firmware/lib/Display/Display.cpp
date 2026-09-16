#include "Display.h"

Display::Display() : u8g2(U8G2_R0, pins::DISPLAY_RST, pins::DISPLAY_SCL, pins::DISPLAY_SDA) {

}