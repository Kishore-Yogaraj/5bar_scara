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
    float _startTicks= 0;
    float _finalTicks= 0;
    float _moveStartTime =0;
    bool  _isMoving = false;   
    float _amax;

    // PWM smoothing
    int _lastPwm;
    int _maxStep;
};