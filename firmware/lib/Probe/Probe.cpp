#include "Probe.h"

Probe::Probe(PiLink& piLink) : piLink(piLink), tempC(0.0f) {}

void Probe::begin() {
    if (oneWire.reset()) {
        oneWire.skip();
        oneWire.write(0x4E);
        oneWire.write(0x7F);
        oneWire.write(0x80);
        oneWire.write(0x7F);
    }
}

void Probe::update() {
    uint32_t now = millis();

    if (!converting && now - lastStart >= config::probe::PERIOD_MS) {
        lastStart = now;
        if (startConversion()) converting = true;
    }

    if (converting && now - lastStart >= config::probe::CONV_MS) {
        converting = false;
        float t;
        if (readTemp(t)) {
            tempC = t;
            wirelink::msg::serial::OwnScalar msg;
            msg.scalar = tempC;

            wirelink::Frame frame;
            if (wirelink::codec::pack(msg, frame)) {
                piLink.send(frame);
            }
        }
    }
}

bool Probe::startConversion() {
    if (!oneWire.reset()) return false;
    oneWire.skip();
    oneWire.write(0x44);
    return true;
}

bool Probe::readTemp(float& temp) {
    uint8_t data[9];
    if (!oneWire.reset()) return false;
    oneWire.skip();
    oneWire.write(0xBE);
    for (int i = 0; i < 9; i++) {
        data[i] = oneWire.read();
    }

    if (OneWire::crc8(data, 8) != data[8]) return false;

    int16_t raw = (data[1] << 8) | data[0];
    if (raw == 0x0550) return false;
    temp = raw / 16.0f;
    return true;
}