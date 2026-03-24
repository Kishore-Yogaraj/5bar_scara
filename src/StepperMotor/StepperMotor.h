#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include <math.h> // for sin/cos

class StepperMotor {
  public:
    StepperMotor(int stepPin, int dirPin, int stepsPerRev);

    void begin();
    void setDirection(bool clockwise); // true = CW, false = CCW
    void moveStepsSinusoidal(int steps, int minDelay, int maxDelay); // steps, fastest delay, slowest delay
    void moveRevolutionsSinusoidal(float revolutions, int minDelay, int maxDelay);

  private:
    int _stepPin;
    int _dirPin;
    int _stepsPerRev;
};

#endif