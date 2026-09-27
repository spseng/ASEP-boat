#pragma once

#include <wirelink/wirelink.h>
#include <ReceivedStates.h>

class PiLink {
public:
    PiLink();

    void begin();

    bool loop();

    bool send(const wirelink::Frame& frame);

    ReceivedStates received_states;
private:
    wirelink::Link link;
};