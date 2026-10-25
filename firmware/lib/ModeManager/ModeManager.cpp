#include "ModeManager.h"
#include <RadioBoat.h>

ModeManager::ModeManager(PiLink& piLink) : piLink(piLink) {
    boat_id = boat::ids::GROUND_STATION_ID; // so that it knows when boat id is valid
    gateState = boat::mode::GateState::TRIPPED;
    initialized = false;
    last_config_timestamp = 0;
    last_gate_enable = boat::mode::GateState::ENABLED;
    fault_flags = static_cast<uint8_t>(boat::mode::fault::NONE);
}

void ModeManager::update() {
    auto timestamp = piLink.received_states.timestamp(wirelink::msg::serial::MsgType::Config);
    if (timestamp.has_value() && timestamp.value() != last_config_timestamp) {
        last_config_timestamp = timestamp.value();
    } else { return; }
    auto entry = piLink.received_states.poll<wirelink::msg::serial::Config>(wirelink::msg::serial::MsgType::Config);

    if (entry.has_value()) {
        if (!update_id(entry->boat_id)) {
            return;
        }
        update_gate_state(static_cast<boat::mode::GateState>(entry->gate_enable));
    }
}

boat::mode::GateState ModeManager::get_gate_state() const {
    return gateState;
}

bool ModeManager::is_initialized() const {
    return initialized;
}

bool ModeManager::update_id(const uint8_t new_id) {
    if (initialized) {
        if (new_id != boat_id) {
            add_fault_flags(static_cast<uint8_t>(boat::mode::fault::ID_CONFLICT));
            return false;
        }
    } else {
        if (boat::ids::is_valid_boat_id(new_id)) {
            boat_id = new_id;
            initialized = true;
        } else {
            add_fault_flags(static_cast<uint8_t>(boat::mode::fault::ID_CONFLICT));
            return false;
        }
    }
    return true;
}

void ModeManager::update_gate_state(const boat::mode::GateState gate_enable) {
    if (last_gate_enable == boat::mode::GateState::TRIPPED && gate_enable == boat::mode::GateState::ENABLED) {
        gateState = boat::mode::GateState::ENABLED;
    }
    last_gate_enable = gate_enable;
}

void ModeManager::send_status(Radio& radio) const {
    wirelink::msg::serial::Status status;
    status.boat_id = boat_id;
    status.gate_state = static_cast<uint8_t>(gateState);
    status.fault_flags = fault_flags;

    const uint32_t now = millis();
    status.links.count = 0;
    for (uint8_t i = 0; i < boat::ids::NUM_IDS; ++i) {
        if (!radio.link_statuses_received[i]) continue;
        auto& link = status.links.items[status.links.count++];
        link.id = i;
        link.rssi = radio.link_statuses[i].rssi;
        link.snr = radio.link_statuses[i].snr;
        link.age_ms = now - radio.link_statuses_last_receive_ms[i];
    }

    wirelink::Frame out_frame;
    if (!wirelink::codec::pack(status, out_frame)) { return; }
    piLink.send(out_frame);
}