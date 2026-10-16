#include "RadioGround.h"

volatile bool Radio::rx_flag = false;
void IRAM_ATTR Radio::onDio1() { rx_flag = true; }

Radio::Radio(GroundLink& groundlink) : groundLink(groundlink) {
    tx_flag = false;
    rx_flag = false;
    may_transmit = false;

    my_tx_id = boat::ids::GROUND_STATION_ID;
    seq = 0;
}

void Radio::begin() {    
    sx1262.tcxoVoltage = config::radio::TCXO_VOLTAGE;

    ConfigLoRa_t loraConfig;
    loraConfig.frequency = config::radio::FREQUENCY;
    loraConfig.bandwidth = config::radio::BANDWIDTH;
    loraConfig.spreadingFactor = config::radio::SPREADING_FACTOR;
    loraConfig.power = config::radio::POWER;

    sx1262.begin(loraConfig);

    sx1262.setDio1Action(onDio1);
    sx1262.startReceive();
    may_transmit = true;
}

void Radio::update() {
    if (rx_flag) {
        rx_flag = false;
        if (tx_flag) {
            tx_flag = false;
            sx1262.finishTransmit();
            sx1262.startReceive();
            last_receive_us = micros();
            last_receive_tx_id = my_tx_id;
            schedule_next_transmit();
        }else {
            receive();
        }
    }
    if (may_transmit && !tx_flag && !rx_flag && (int32_t)(micros() - my_next_transmit_us) >= 0) {
        transmit();
    }
}

void Radio::receive() {
    size_t len = sx1262.getPacketLength();
    uint8_t buf[255];
    int st = sx1262.readData(buf, len);
    last_receive_us = micros() - sx1262.getTimeOnAir(len);
    if (st == RADIOLIB_ERR_NONE) {
        wirelink::bytes::Packet packet;
        if (len > packet.data.size()) { return; }
        std::copy(buf, buf + len, packet.data.begin());
        packet.len = len;

        auto frame = wirelink::framing::unwrap(packet);
        
        if (frame) {
            auto msg_type = static_cast<wirelink::msg::lora::MsgType>(frame->type);
            if (msg_type == wirelink::msg::lora::MsgType::SelfStatus) {
                wirelink::msg::lora::SelfStatus status;
                if (wirelink::codec::unpack(*frame, status)) {
                    last_receive_tx_id = status.self.id;
                    groundLink.send(packet);
                }
            } else { return; } // boats should only transmit SelfStatus

            may_transmit = true;
            schedule_next_transmit();
        }
    }
}

void Radio::schedule_next_transmit() {
    uint8_t delta_id = (boat::ids::NUM_IDS + my_tx_id - last_receive_tx_id) % boat::ids::NUM_IDS;
    if (delta_id == 0) delta_id = boat::ids::NUM_IDS;
    my_next_transmit_us = last_receive_us + delta_id * boat::lora::config::TRANSMIT_INTERVAL;
}

void Radio::transmit() {
    if (groundLink.is_inbox_empty()) {
        wirelink::msg::lora::Heartbeat heartbeat;
        wirelink::Frame frame;
        if (!wirelink::codec::pack(heartbeat, frame)) { return; }
        auto packet = wirelink::framing::wrap(frame, seq);
        if (!packet) { return; }
        seq++;
        
        int st = sx1262.transmit(packet->data.data(), packet->len);
        if (st == RADIOLIB_ERR_NONE) {
            tx_flag = true;
        } else {
            sx1262.startReceive();
            last_receive_us = micros();
            last_receive_tx_id = my_tx_id;
            schedule_next_transmit();
        }
        return;
    } else {
        auto packet = groundLink.get_last_received_packet();
        
        int st = sx1262.transmit(packet.data.data(), packet.len);
        if (st == RADIOLIB_ERR_NONE) {
            tx_flag = true;
        } else {
            sx1262.startReceive();
            last_receive_us = micros();
            last_receive_tx_id = my_tx_id;
            schedule_next_transmit();
        }

        groundLink.echo_lora_transmission(packet);
        return;
    }
}