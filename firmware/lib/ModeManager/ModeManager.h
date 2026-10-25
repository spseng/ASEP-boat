#pragma once

#include <boat_defs/boat_defs.h>
#include <PiLink.h>

class Radio;

class ModeManager {
public:
    ModeManager(PiLink& piLink);

    void update();

    void trip_gate() { gateState = boat::mode::GateState::TRIPPED; }

    boat::mode::GateState get_gate_state() const;

    bool is_initialized() const;

    uint8_t get_boat_id() const { return boat_id; }

    uint8_t get_fault_flags() const { return fault_flags; }

    void add_fault_flags(const uint8_t new_fault_flag) { fault_flags |= new_fault_flag; gateState = boat::mode::GateState::TRIPPED; }

    void clear_fault_flags(const uint8_t clear_fault_flag) { fault_flags &= ~clear_fault_flag; }

    void send_status(Radio& radio) const;

private:
    PiLink& piLink;

    uint8_t boat_id;

    boat::mode::GateState gateState;

    bool initialized = false;

    uint32_t last_config_timestamp;
    boat::mode::GateState last_gate_enable;

    uint8_t fault_flags;

    bool update_id(const uint8_t new_id);
    void update_gate_state(const boat::mode::GateState gate_enable);
};