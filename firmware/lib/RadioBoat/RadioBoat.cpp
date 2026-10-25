#include "RadioBoat.h"

volatile bool Radio::rx_flag = false;
void IRAM_ATTR Radio::onDio1() { rx_flag = true; }

Radio::Radio(PiLink& piLink, ModeManager& modeManager) : piLink(piLink), modeManager(modeManager) {
    tx_flag = false;
    rx_flag = false;
    may_transmit = false;
    seq = 0;

    link_statuses.fill(wirelink::msg::common::LinkStatus{});
    for (size_t i = 0; i < link_statuses.size(); ++i) {
        link_statuses[i].id = i;
    }
    link_statuses_received.fill(false);
    link_statuses_last_receive_ms.fill(0);
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
                last_receive_tx_id = boat::ids::GROUND_STATION_ID;
                wirelink::msg::lora::Disable disable;
                if (wirelink::codec::unpack(*frame, disable)) {
                    if (disable.rx_id == my_tx_id || disable.rx_id == boat::ids::BROADCAST_ID) {
                        modeManager.trip_gate();
                    }
                } else { return; }
            }else if (msg_type == wirelink::msg::lora::MsgType::SelfStatus) {
                wirelink::msg::lora::SelfStatus status;
                if (wirelink::codec::unpack(*frame, status)) {
                    last_receive_tx_id = status.self.id;
                    update_peer_map(last_receive_tx_id, status.self.lat, status.self.lon, status.self.scalar, status.self.heading_deg);
                } else { return; }
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

            update_link_status(last_receive_tx_id, sx1262.getRSSI(), sx1262.getSNR());

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
        status.self.lat = 0;
        status.self.lon = 0;
        status.self.scalar = std::numeric_limits<float>::quiet_NaN();
        status.self.heading_deg = std::numeric_limits<float>::quiet_NaN();
    }

    auto heartbeat = piLink.received_states.poll<wirelink::msg::serial::Heartbeat>(wirelink::msg::serial::MsgType::Heartbeat);
    if (heartbeat) {
        status.state = wirelink::msg::lora::make_state(
            heartbeat->ros_mode,
            heartbeat->ros_armed,
            static_cast<uint8_t>(modeManager.get_gate_state())
        );
        status.faults = modeManager.get_fault_flags();
    } else {
        status.state = wirelink::msg::lora::make_state(
            static_cast<uint8_t>(boat::mode::Mode::MANUAL),
            static_cast<uint8_t>(boat::mode::ArmedState::DISARMED),
            static_cast<uint8_t>(modeManager.get_gate_state())
        );
        status.faults = modeManager.get_fault_flags();
    }

    if (link_statuses_received[boat::ids::GROUND_STATION_ID]) {
        status.gs_rssi = link_statuses[boat::ids::GROUND_STATION_ID].rssi;
        status.gs_snr = link_statuses[boat::ids::GROUND_STATION_ID].snr;
    } else {
        status.gs_rssi = std::numeric_limits<float>::quiet_NaN();
        status.gs_snr = std::numeric_limits<float>::quiet_NaN();
    }

    wirelink::Frame frame;
    if (!wirelink::codec::pack(status, frame)) { return; }
    auto packet = wirelink::framing::wrap(frame, seq++);
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

void Radio::update_link_status(const uint8_t id, const float rssi, const float snr) {
    if (id >= boat::ids::NUM_IDS) { return; }
    link_statuses[id].rssi = rssi;
    link_statuses[id].snr = snr;
    link_statuses_received[id] = true;
    link_statuses_last_receive_ms[id] = millis();
}

void Radio::update_peer_map(const uint8_t id, const int32_t lat, const int32_t lon, const float scalar, const float heading_deg) {
    if (!boat::ids::is_valid_boat_id(id)) { return; }
    peer_map[id].id = id;
    peer_map[id].lat = lat;
    peer_map[id].lon = lon;
    peer_map[id].scalar = scalar;
    peer_map[id].heading_deg = heading_deg;
}