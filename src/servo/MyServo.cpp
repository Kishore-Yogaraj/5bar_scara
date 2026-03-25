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
}

// Logic implementation
void MyServo::writeAngle(int angle) {
    angle = constrain(angle, 0, 180);
    
    // Map angle to pulse width
    long us = map(angle, 0, 180, _minUs, _maxUs);
    
    // Convert us to 16-bit duty cycle (20000us = 50Hz period)
    uint32_t duty = (us * 65535) / 20000;
    
    ledcWrite(_channel, duty);
}