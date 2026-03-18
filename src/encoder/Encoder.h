#pragma once
#include "driver/pcnt.h"

class Encoder {
public:
    Encoder(int pinA, int pinB, pcnt_unit_t unit);
    void begin();
    int  getPosition() const;
    void reset();

private:
    int          _pinA, _pinB;
    pcnt_unit_t  _unit;
};