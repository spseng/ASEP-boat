#include "PiLink.h"

#include "config.h"
#include <Arduino.h>
#include <boat_defs/boat_defs.h>

PiLink::PiLink() : link(), received_states() {}

void PiLink::begin() {
    Serial.begin(boat::serial::BAUD_RATE);
}

bool PiLink::update() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        auto frame = link.feed(b);
        if (frame) {
            received_states.cache(*frame, millis());
        }
    }
    return true;
}

bool PiLink::send(const wirelink::Frame& frame) {
    auto packet = link.send(frame);
    if (!packet) return false;
    Serial.write(packet->data.data(), packet->len);
    return true;
}