#include "DCMotor.h"
#include "config.h"
#include <Arduino.h>

DCMotor::DCMotor(int rpwmPin, int lpwmPin,
                 uint8_t rpwmCh, uint8_t lpwmCh,
                 Encoder& encoder, PIDController& pid)
    : _rpwmPin(rpwmPin), _lpwmPin(lpwmPin),
      _rpwmCh(rpwmCh),   _lpwmCh(lpwmCh),
      _encoder(encoder),  _pid(pid),
      _targetTicks(0.0f), _prevTime(0) {}

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
    _targetTicks = (degrees / 360.0f) * CPR;
    _pid.reset();
}

void DCMotor::update() {
    unsigned long now = millis();
    float dt = (now - _prevTime) / 1000.0f;
    if (dt < 0.001f) return;
    _prevTime = now;

    float error  = _targetTicks - _encoder.getPosition();
    float output = _pid.compute(error, dt);

    if (output == 0.0f) {
        ledcWrite(_rpwmCh, 0);
        ledcWrite(_lpwmCh, 0);
        return;
    }

    int pwm = (int)constrain(abs(output), 0, PWM_MAX);
    if (output > 0) {
        ledcWrite(_rpwmCh, pwm);
        ledcWrite(_lpwmCh, 0);
    } else {
        ledcWrite(_rpwmCh, 0);
        ledcWrite(_lpwmCh, pwm);
    }

}