#pragma once
#include <Arduino.h>

struct MotorCommand {
    float angle1;
    float angle2;
    bool  valid;
    bool  home;
};

class SerialComms {
public:
    SerialComms(int baudRate);

    void         begin();
    MotorCommand read();

private:
    int _baudRate;
};