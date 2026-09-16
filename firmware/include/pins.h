#pragma once
#include <cstdint>

namespace pins {
    constexpr uint8_t RADIO_CS = 8;
    constexpr uint8_t RADIO_SCK = 9;
    constexpr uint8_t RADIO_MOSI = 10;
    constexpr uint8_t RADIO_MISO = 11;
    constexpr uint8_t RADIO_RST = 12;
    constexpr uint8_t RADIO_BUSY = 13;
    constexpr uint8_t RADIO_DIO1 = 14;

    constexpr uint8_t DISPLAY_SDA = 17;
    constexpr uint8_t DISPLAY_SCL = 18;
    constexpr uint8_t DISPLAY_RST = 21;

    constexpr uint8_t VEXT = 36; //on when LOW

    constexpr uint8_t LED = 35;
    constexpr uint8_t PRG_BTN = 0;
}