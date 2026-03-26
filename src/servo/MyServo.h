#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>

class MyServo {
private:
    int _pin;
    int _channel;

    // MG90S specific constants
    const int _freq       = 50;
    const int _resolution = 16;
    const int _minUs      = 500;
    const int _maxUs      = 2500;

    float _currentAngle;
    float _targetAngle;
    float _speed = 50.0f;   // deg/sec

    unsigned long _lastTime;

public:
    // Constructor (default channel 0)
    MyServo(int pin, int channel = 0);

    // Initialize hardware
    void begin();

    // Direct write (instant, no slewing)
    void writeAngle(int angle);

    // Smooth motion API
    void setTargetAngle(int angle);
    void update();
    bool isAtTarget();
};

#endif