# pragma once

class PIDController{
public:
    PIDController(float kp, float ki, float kd, float integralLimit, float deadband);
    float compute(float error, float dt);
    void reset();
    void setGains(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _integralLimit;
    float _deadband;
    float _integral;
    float _prevError;
};