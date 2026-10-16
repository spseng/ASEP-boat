#pragma once

#include <wirelink/wirelink.h>

class GroundLink {
public:
    GroundLink();

    void begin();

    bool update();

    bool send(wirelink::bytes::Packet& packet);

    bool echo_lora_transmission(wirelink::bytes::Packet& packet);

    wirelink::bytes::Packet get_last_received_packet() const { return last_received_packet; }

    bool is_inbox_empty() const { return !has_received_packet; }

private:
    wirelink::Link link;

    wirelink::bytes::Packet last_received_packet;
    bool has_received_packet;
};