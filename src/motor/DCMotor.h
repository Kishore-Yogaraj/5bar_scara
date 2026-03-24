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
    void setPID(float kp_val,float ki_val, float kd_val);

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
    float _vmax;  
    float _amax;

    // PWM smoothing
    int _lastPwm;
    int _maxStep;

    // test ing different PID
    float error = 0;
    float prev_error = 0;
    float integral = 0;
    float derivative = 0;

    float output = 0;
    float targetPosition = 0;
    float dt = 0;
    unsigned long prevTime = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
};