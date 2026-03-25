#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>

class MyServo {
private:
    int _pin;
    int _channel;
    
    // MG90S specific constants
    const int _freq = 50; 
    const int _resolution = 16;
    const int _minUs = 500; 
    const int _maxUs = 2500;

public:
    // Constructor (Default channel set here)
    MyServo(int pin, int channel = 0);

    // Initialize hardware
    void begin();

    // Move to angle
    void writeAngle(int angle);
};

#endif