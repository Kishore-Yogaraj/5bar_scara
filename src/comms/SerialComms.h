#pragma once
#include <Arduino.h>

struct MotorCommand {
    float angle1;
    float angle2;
    int   servo_angle;
    bool  valid;
    bool  home;
    bool  rotate;        // rotate stepper to pick zone (CW 45°)
    bool  rotate_home;   // rotate stepper back to drop zone (CCW 45°)
    bool  pick;          // execute gripper pick sequence
    bool  drop;          // execute gripper drop sequence
    bool  step_cmd;      // move stepper by stepper_deg degrees (+ = CW, - = CCW)
    float stepper_deg;
};

class SerialComms {
public:
    SerialComms(int baudRate);

    void         begin();
    MotorCommand read();

private:
    int _baudRate;
};