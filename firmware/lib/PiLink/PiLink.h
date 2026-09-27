#pragma once

#include <wirelink/wirelink.h>
#include <ReceivedStates.h>

class PiLink {
public:
    PiLink();

    void begin();

    bool update();

    bool send(const wirelink::Frame& frame);

    ReceivedStates received_states;
private:
    wirelink::Link link;
};