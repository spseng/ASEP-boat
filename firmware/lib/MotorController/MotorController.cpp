#include "MotorController.h"

#include "config.h"
#include "pins.h"

MotorController::MotorController(PiLink& piLink) : piLink(piLink) {}

void MotorController::begin() {
    pwm_begin();
}

bool MotorController::make_channel(uint8_t gpio, mcpwm_cmpr_handle_t* cmpr) {
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operCfg = {};
    operCfg.group_id = 0;
    if (mcpwm_new_operator(&operCfg, &oper) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(oper, s_timer) != ESP_OK) return false;

    mcpwm_cmpr_handle_t cmp = NULL;
    mcpwm_comparator_config_t cmpCfg = {};
    cmpCfg.flags.update_cmp_on_tez = true;      // take a new width at frame start
    if (mcpwm_new_comparator(oper, &cmpCfg, &cmp) != ESP_OK) return false;

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t genCfg = {};
    genCfg.gen_gpio_num = gpio;
    if (mcpwm_new_generator(oper, &genCfg, &gen) != ESP_OK) return false;

    if (mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)) != ESP_OK) return false;
    if (mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp,
                                       MCPWM_GEN_ACTION_LOW)) != ESP_OK) return false;

    mcpwm_comparator_set_compare_value(cmp, NEUTRAL_US);
    *cmpr = cmp;
    return true;
}

bool MotorController::pwm_begin() {
    mcpwm_timer_config_t timerCfg = {};
    timerCfg.group_id      = 0;
    timerCfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timerCfg.resolution_hz = 1000000;      // 1 MHz -> 1 tick = 1 us
    timerCfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    timerCfg.period_ticks  = 20000;        // 20 ms frame = 50 Hz
    if (mcpwm_new_timer(&timerCfg, &s_timer) != ESP_OK) {
        return false;
    }
    if (!make_channel(pins::ESC_PORT, &s_cmprPort)) { return false; }
    if (!make_channel(pins::ESC_STBD, &s_cmprStbd)) { return false; }

    if (mcpwm_timer_enable(s_timer) != ESP_OK) return false;
    if (mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;
    return true;
}

void MotorController::pwm_write(uint16_t port, uint16_t stbd) {
    mcpwm_comparator_set_compare_value(s_cmprPort, port);
    mcpwm_comparator_set_compare_value(s_cmprStbd, stbd);
}