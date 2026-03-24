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
    return _targetTicks;
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
    _startTicks = _encoder.getPosition();
    _finalTicks = (degrees / 360.0f) * CPR;

    _moveStartTime = millis() / 1000.0f;
    _vmax = 3000.0f;   // tune this
    _amax = 6000.0f;   // tune this

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
    float currentTime = millis() / 1000.0f;
    float dt = (currentTime - _prevTime);
    if (dt < 0.005f) return;
    _prevTime = currentTime;

    
    float t = currentTime - _moveStartTime;

    float distance = _finalTicks - _startTicks;
    float dir = (distance >= 0) ? 1.0f : -1.0f;
    distance = abs(distance);

    // time to accelerate
    float t_acc = _vmax / _amax;

    // distance during accel
    float d_acc = 0.5f * _amax * t_acc * t_acc;

    // check if triangular (short move)
    float t_total;

    if (2 * d_acc > distance) {
        // triangular profile
        t_acc = sqrt(distance / _amax);
        t_total = 2 * t_acc;
    } else {
        float d_const = distance - 2 * d_acc;
        float t_const = d_const / _vmax;
        t_total = 2 * t_acc + t_const;
    }

    float refPos;

    if (t >= t_total) {
        refPos = _finalTicks;
        _isMoving = false;
    } else if (t < t_acc) {
        // acceleration phase
        refPos = _startTicks + dir * (0.5f * _amax * t * t);
    } else if (t < (t_total - t_acc)) {
        // constant velocity
        float t1 = t - t_acc;
        refPos = _startTicks + dir * (d_acc + _vmax * t1);
    } else {
        // deceleration
        float t2 = t - (t_total - t_acc);
        refPos = _finalTicks - dir * (0.5f * _amax * (t_acc - t2) * (t_acc - t2));
    }
    
    float error =  refPos - _encoder.getPosition();
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