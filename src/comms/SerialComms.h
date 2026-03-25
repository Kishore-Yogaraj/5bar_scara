#pragma once
#include <Arduino.h>

struct MotorCommand {
    float angle1;
    float angle2;
    int   servo_angle;
    bool  valid;
    bool  home;
    bool  rotate;   // triggers 180° stepper repositioning
};

class SerialComms {
public:
    SerialComms(int baudRate);

    void         begin();
    MotorCommand read();

private:
    int _baudRate;
};