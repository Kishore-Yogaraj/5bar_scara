#ifndef GRIPPER_H
#define GRIPPER_H

#include "servo/MyServo.h"

class Gripper {
public:
    Gripper(MyServo& grip, MyServo& lift);
    void begin(int gOpen, int gClosed, int lUp, int lDown);

    // Non-blocking sequences — call pickSequence()/dropSequence() once to start,
    // then call update() every loop() until isRunningPick()/isRunningDrop() is false.
    void pickSequence();
    void dropSequence();
    void update();

    bool isRunningPick();
    bool isRunningDrop();

private:
    MyServo& _gripServo;
    MyServo& _liftServo;

    int _gripOpen, _gripClosed;
    int _liftUp, _liftDown;

    enum PickState  { P_IDLE, P_LOWERING, P_GRIPPING, P_LIFTING, P_LIFTING_DONE };
    enum DropState  { D_IDLE, D_LOWERING, D_OPENING,  D_LIFTING, D_LIFTING_DONE };

    PickState _pickState = P_IDLE;
    DropState _dropState = D_IDLE;

    bool _runningPick = false;
    bool _runningDrop = false;
};

#endif
