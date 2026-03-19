#include "SerialComms.h"

SerialComms::SerialComms(int baudRate)
    : _baudRate(baudRate) {}

void SerialComms::begin() {
    Serial.begin(_baudRate);
}

MotorCommand SerialComms::read() {
    MotorCommand cmd = {0.0f, 0.0f, false, false};

    if (!Serial.available()) {
        return cmd;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "HOME") {
        cmd.home = true;
        return cmd;
    }

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
        return cmd;     // malformed, no comma found
    }

    cmd.angle1 = input.substring(0, commaIndex).toFloat();
    cmd.angle2 = input.substring(commaIndex + 1).toFloat();
    cmd.valid  = true;

    return cmd;
}