#include "StepperMotor.h"

StepperMotor::StepperMotor(int stepPin, int dirPin, int stepsPerRev) {
    _stepPin = stepPin;
    _dirPin = dirPin;
    _stepsPerRev = stepsPerRev;
}

void StepperMotor::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, HIGH); // default clockwise
}

void StepperMotor::setDirection(bool clockwise) {
    digitalWrite(_dirPin, clockwise ? HIGH : LOW);
}

void StepperMotor::moveStepsSinusoidal(int steps, int minDelay, int maxDelay) {
    // minDelay = fastest speed (us), maxDelay = slowest speed (us)
    for (int i = 0; i < steps; i++) {
        // Sinusoidal profile: starts slow, speeds up, slows down
        float phase = (float)i / steps * PI;  // 0 → PI
        int delayTime = int(minDelay + (maxDelay - minDelay) * (0.5 * (1 - cos(phase))));
        
        // STEP pulse
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(delayTime);
    }
}

void StepperMotor::moveAngleSinusoidal(float angles, int minDelay, int maxDelay) {
    int steps = int((angles/360) * _stepsPerRev);
    moveStepsSinusoidal(steps, minDelay, maxDelay);
}