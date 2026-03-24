#include "DCMotor.h"
#include "config.h"
#include <Arduino.h>

DCMotor::DCMotor(int rpwmPin, int lpwmPin,
                 uint8_t rpwmCh, uint8_t lpwmCh,
                 Encoder& encoder, PIDController& pid)
    : _rpwmPin(rpwmPin), _lpwmPin(lpwmPin),
      _rpwmCh(rpwmCh),   _lpwmCh(lpwmCh),
      _encoder(encoder),  _pid(pid),
      _targetTicks(0.0f), _prevTime(0),
      _startTicks(0.0f), _finalTicks(0.0f),
      _moveStartTime(0.0f),
      _isMoving(false),
      _lastPwm(0), _maxStep(5) {}

float DCMotor::getTargetTicks() const{
    return _finalTicks;
}

void DCMotor::begin() {
    pinMode(_rpwmPin, OUTPUT);
    pinMode(_lpwmPin, OUTPUT);
    digitalWrite(_rpwmPin, LOW);
    digitalWrite(_lpwmPin, LOW);

    ledcSetup(_rpwmCh, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_rpwmPin, _rpwmCh);
    ledcSetup(_lpwmCh, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_lpwmPin, _lpwmCh);

    _encoder.begin();
    _prevTime = millis();
}

void DCMotor::setTargetDegrees(float degrees) {

    float currentPos = _encoder.getPosition();
    // float newTarget  = currentPos + (degrees / 360.0f) * CPR;  // relative move

    // // ignore tiny moves
    // if (abs(newTarget - currentPos) < 5.0f) {
    //     _isMoving = false;
    //     return;
    // }
    _startTicks = currentPos;
    _finalTicks = (degrees / 360.0f) * CPR;
    

    _moveStartTime = millis() / 1000.0f;

    _amax = 100.0f;   // tune this

    _isMoving = true;

    _pid.reset();
}

void DCMotor::update() {
    // if (!_isMoving) {
    //     ledcWrite(_rpwmCh, 0);
    //     ledcWrite(_lpwmCh, 0);
    //     _lastPwm = 0;
    //     return;
    // }

    float currentTime = millis() / 1000.0f;          // current time in seconds
    float dt = currentTime - _prevTime;             // time since last update
    if (dt < 0.005f) return;                        // skip update if too fast
    _prevTime = currentTime;

    float t = currentTime - _moveStartTime;         // time since motion started

    float distance = _finalTicks - _startTicks;    // total movement in ticks
    float dir = (distance >= 0) ? 1.0f : -1.0f;    // movement direction
    distance = abs(distance);

    // triangular profile: acceleration = deceleration
    float t_acc = sqrt(distance / _amax);           // time to reach peak velocity
    float t_total = 2.0f * t_acc;                  // total motion time

    float refPos;

    // Determine reference position along the triangle
    if (t >= t_total) {
        // finished motion
        refPos = _finalTicks;
        _isMoving = false;
    } else if (t < t_acc) {
        // acceleration phase
        refPos = _startTicks + dir * 0.5f * _amax * t * t;
    } else {
        // deceleration phase
        float t2 = t - t_acc;
        refPos = _finalTicks - dir * 0.5f * _amax * (t_acc - t2) * (t_acc - t2);
    }

    // PID error and output
    float error = refPos - _encoder.getPosition();
    float output = _pid.compute(error, dt);

    
    int maxStep = 5;

    if (!_isMoving && abs(error) < 10) {
        ledcWrite(_rpwmCh, 0);
        ledcWrite(_lpwmCh, 0);
        _lastPwm = 0;
        _pid.reset();
        return;
    }

    
   int targetPwm = (int)constrain(abs(output), 0, 200);

   int minPwm = 60;

    if (_isMoving && abs(error) > 50 && targetPwm < minPwm) {
        targetPwm = minPwm;
    }

    // Slew rate limit
    int pwm;
    if (targetPwm > _lastPwm)
        pwm = min(_lastPwm + _maxStep, targetPwm);
    else
        pwm = max(_lastPwm - _maxStep, targetPwm);

    _lastPwm = pwm;

    if (output > 0) {
        ledcWrite(_rpwmCh, pwm);
        ledcWrite(_lpwmCh, 0);
    } else {
        ledcWrite(_rpwmCh, 0);
        ledcWrite(_lpwmCh, pwm);
    }
    if (distance < 1.0f) {
        ledcWrite(_rpwmCh, 0);
        ledcWrite(_lpwmCh, 0);
        return;
    }

}