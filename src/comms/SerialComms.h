#pragma once
#include <Arduino.h>

struct MotorCommand {
    float angle1;
    float angle2;

    float Kp1, Ki1, Kd1;
    float Kp2, Ki2, Kd2;

    bool  valid;
    bool  home;
    bool pidUpdate;
};

class SerialComms {
public:
    SerialComms(int baudRate);

    void         begin();
    MotorCommand read();

private:
    int _baudRate;
};