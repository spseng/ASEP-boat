#include "RadioBoat.h"

volatile bool Radio::rx_flag = false;
void IRAM_ATTR Radio::onDio1() { rx_flag = true; }

Radio::Radio(PiLink& piLink) : piLink(piLink) {
    tx_flag = false;
    rx_flag = false;
    may_transmit = false;
}

void Radio::begin(uint8_t my_tx_id) {
    this->my_tx_id = my_tx_id;
    
    sx1262.tcxoVoltage = config::radio::TCXO_VOLTAGE;

    ConfigLoRa_t loraConfig;
    loraConfig.frequency = config::radio::FREQUENCY;
    loraConfig.bandwidth = config::radio::BANDWIDTH;
    loraConfig.spreadingFactor = config::radio::SPREADING_FACTOR;
    loraConfig.power = config::radio::POWER;

    sx1262.begin(loraConfig);

    sx1262.setDio1Action(onDio1);
    sx1262.startReceive();
    may_transmit = false;
}

void Radio::update() {
    if (rx_flag) {
        rx_flag = false;
        if (tx_flag) {
            tx_flag = false;
            sx1262.finishTransmit();
            sx1262.startReceive();
            last_receive_us = my_next_transmit_us;
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
            if (msg_type == wirelink::msg::lora::MsgType::Disable) {
                //TODO: Implement
            }else if (msg_type == wirelink::msg::lora::MsgType::SelfStatus) {
                wirelink::msg::lora::SelfStatus status;
                if (wirelink::codec::unpack(*frame, status)) {
                    last_receive_tx_id = status.self.id;
                    // TODO: map peer status
                }
            }
            else {
                last_receive_tx_id = boat::ids::GROUND_STATION_ID;
                wirelink::msg::serial::ReceivedCommand command;
                command.lora_type = frame->type;
                command.payload = frame->payload;
                command.payload.len = frame->len;
                wirelink::Frame out_frame;
                if (wirelink::codec::pack(command, out_frame)) {
                    piLink.send(out_frame);
                }
            }

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
    wirelink::msg::lora::SelfStatus status;
    auto entry = piLink.received_states.poll<wirelink::msg::serial::BroadcastEntry>(wirelink::msg::serial::MsgType::BroadcastEntry);
    if (entry) {
        status.self = entry->self;
    } else {
        status.self.id = my_tx_id;
        status.self.lat = 0; // TODO: Implement
        status.self.lon = 0; // TODO: Implement
        status.self.scalar = 0.0; // TODO: Implement
    }
    status.heading_deg = 0.0; // TODO: Implement
    status.state = wirelink::msg::lora::make_state(
    static_cast<uint8_t>(boat::mode::Mode::MANUAL),
    static_cast<uint8_t>(boat::mode::ArmedState::ARMED),
    static_cast<uint8_t>(boat::mode::GateState::ENABLED)); // TODO: Implement
    status.faults = 0; // TODO: Implement
    status.gs_rssi = 0.0; // TODO: Implement
    status.gs_snr = 0.0; // TODO: Implement

    wirelink::Frame frame;
    if (!wirelink::codec::pack(status, frame)) { return; }
    auto packet = wirelink::framing::wrap(frame, 0);
    if (!packet) { return; }

    int st = sx1262.startTransmit(packet->data.data(), packet->len);
    if (st == RADIOLIB_ERR_NONE) {
        tx_flag = true;
    } else {
        sx1262.startReceive();
        last_receive_us = micros();
        last_receive_tx_id = my_tx_id;
        schedule_next_transmit();
    }
}