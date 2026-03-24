#pragma once
#include "encoder/Encoder.h"
#include "pid/PIDController.h"

class DCMotor {
public:
    DCMotor(int rpwmPin, int lpwmPin,
            uint8_t rpwmCh, uint8_t lpwmCh,
            Encoder& encoder,
            PIDController& pid);

    void begin();
    void setTargetDegrees(float degrees);
    void update();
    float getTargetTicks() const;

private:
    int     _rpwmPin, _lpwmPin;
    uint8_t _rpwmCh, _lpwmCh;

    Encoder&       _encoder;
    PIDController& _pid;

    float         _targetTicks;
    unsigned long _prevTime;

    // Trajectory control
    float _startTicks;
    float _finalTicks;
    float _moveStartTime;
    bool  _isMoving;
    float _vmax;    
    float _amax;

    // PWM smoothing
    int _lastPwm;
    int _maxStep;
};