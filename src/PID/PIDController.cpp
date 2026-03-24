#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float integralLimit, float deadband)
    :_kp(kp), _ki(ki), _kd(kd), _integralLimit(integralLimit), _deadband(deadband), _integral(0.0f), _prevError(0.0f){}

float PIDController::compute(float error, float dt){
    if (abs(error) < _deadband){
        //reset();
        return 0.0f;
    }

    _integral += error * dt;
    _integral = constrain(_integral, -_integralLimit, _integralLimit);

    //float derivative = (error - _prevError) / dt;
    float derivative = (dt > 0.01f) ? (error - _prevError) / dt : 0.0f;
    _prevError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
    
}

void PIDController::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}