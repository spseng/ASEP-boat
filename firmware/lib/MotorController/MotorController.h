#pragma once

#include <PiLink.h>
#include "driver/mcpwm_prelude.h"

class MotorController {
public:
    MotorController(PiLink& piLink);

    void begin();
    void update();
private:
    PiLink& piLink;
    
    static mcpwm_timer_handle_t s_timer;
    static mcpwm_cmpr_handle_t s_cmprPort, s_cmprStbd;

    bool make_channel(uint8_t gpio, mcpwm_cmpr_handle_t* cmpr);

    bool pwm_begin();

    void pwm_write(uint16_t port, uint16_t stbd);
};