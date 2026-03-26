#include "Gripper.h"

Gripper::Gripper(MyServo& grip, MyServo& lift)
    : _gripServo(grip), _liftServo(lift) {}

void Gripper::begin(int gOpen, int gClosed, int lUp, int lDown) {
    _gripOpen   = gOpen;
    _gripClosed = gClosed;
    _liftUp     = lUp;
    _liftDown   = lDown;

    _gripServo.begin();
    _liftServo.begin();

    _gripServo.setTargetAngle(_gripOpen);
    _liftServo.setTargetAngle(_liftUp);
}

void Gripper::pickSequence() { _runningPick = true; }
void Gripper::dropSequence() { _runningDrop = true; }

void Gripper::update() {
    _gripServo.update();
    _liftServo.update();

    // Pick: lower → close → lift
    if (_runningPick) {
        switch (_pickState) {
            case P_IDLE:
                _liftServo.setTargetAngle(_liftDown);
                _pickState = P_LOWERING;
                break;
            case P_LOWERING:
                if (_liftServo.isAtTarget()) _pickState = P_CLOSING;
                break;
            case P_CLOSING:
                _gripServo.setTargetAngle(_gripClosed);
                _pickState = P_LIFTING;
                break;
            case P_LIFTING:
                if (_gripServo.isAtTarget()) {
                    _liftServo.setTargetAngle(_liftUp);
                    _pickState   = P_IDLE;
                    _runningPick = false;
                }
                break;
        }
    }

    // Drop: lower → open → lift
    if (_runningDrop) {
        switch (_dropState) {
            case D_IDLE:
                _liftServo.setTargetAngle(_liftDown);
                _dropState = D_LOWERING;
                break;
            case D_LOWERING:
                if (_liftServo.isAtTarget()) _dropState = D_OPENING;
                break;
            case D_OPENING:
                _gripServo.setTargetAngle(_gripOpen);
                _dropState = D_LIFTING;
                break;
            case D_LIFTING:
                if (_gripServo.isAtTarget()) {
                    _liftServo.setTargetAngle(_liftUp);
                    _dropState   = D_IDLE;
                    _runningDrop = false;
                }
                break;
        }
    }
}

bool Gripper::isRunningPick() { return _runningPick; }
bool Gripper::isRunningDrop() { return _runningDrop; }
