#include "GroundLink.h"

#include "config.h"
#include <Arduino.h>

GroundLink::GroundLink() : link(), last_received_packet(), has_received_packet(false) {}

void GroundLink::begin() {
    Serial.begin(config::ground_link::BAUD_RATE);
}

bool GroundLink::update() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        auto frame = link.feed(b);
        if (frame) {
            auto packet = wirelink::framing::wrap(*frame, 0);
            if (!packet) { return false; }
            last_received_packet = *packet;
            has_received_packet = true;
        }
    }
    return true;
}

bool GroundLink::send(wirelink::bytes::Packet& packet) {
    Serial.write(packet.data.data(), packet.len);
    return true;
}

bool GroundLink::echo_lora_transmission(wirelink::bytes::Packet& packet) {
    Serial.write(packet.data.data(), packet.len);
    has_received_packet = false;
    return true;
}