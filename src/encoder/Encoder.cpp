#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB, pcnt_unit_t unit)
    : _pinA(pinA), _pinB(pinB), _unit(unit) {}

void Encoder::begin() {
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = _pinA;
    cfg.ctrl_gpio_num  = _pinB;
    cfg.unit           = _unit;
    cfg.channel        = PCNT_CHANNEL_0;
    cfg.pos_mode       = PCNT_COUNT_INC;
    cfg.neg_mode       = PCNT_COUNT_DEC;
    cfg.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.counter_h_lim  =  32767;
    cfg.counter_l_lim  = -32768;
    pcnt_unit_config(&cfg);

    pcnt_set_filter_value(_unit, 10);
    pcnt_filter_enable(_unit);
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);
}

int Encoder::getPosition() const {
    int16_t count;
    pcnt_get_counter_value(_unit, &count);
    return count;
}

void Encoder::reset() {
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);
}