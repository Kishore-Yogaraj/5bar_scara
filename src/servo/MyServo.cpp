#include "MyServo.h"

// Constructor implementation
MyServo::MyServo(int pin, int channel) {
    _pin = pin;
    _channel = channel;
}

// Setup implementation
void MyServo::begin() {
    ledcSetup(_channel, _freq, _resolution);
    ledcAttachPin(_pin, _channel);
    _currentAngle = 0.0f;
    _targetAngle  = 0.0f;
    _lastTime     = millis();
}

// Direct write (instant, no slewing)
void MyServo::writeAngle(int angle) {
    angle = constrain(angle, 0, 180);

    long us = map(angle, 0, 180, _minUs, _maxUs);
    uint32_t duty = (us * 65535) / 20000;
    ledcWrite(_channel, duty);
}

// Set smooth-motion target
void MyServo::setTargetAngle(int angle) {
    _targetAngle = constrain(angle, 0, 180);
}

// Call every loop() — slews currentAngle toward targetAngle at _speed deg/s
void MyServo::update() {
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0f;
    _lastTime = now;

    float error   = _targetAngle - _currentAngle;
    float maxStep = _speed * dt;

    if (abs(error) <= maxStep) {
        _currentAngle = _targetAngle;
    } else {
        _currentAngle += (error > 0 ? maxStep : -maxStep);
    }

    writeAngle((int)_currentAngle);
}

bool MyServo::isAtTarget() {
    return abs(_currentAngle - _targetAngle) < 1.0f;
}