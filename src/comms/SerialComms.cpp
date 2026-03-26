#include "SerialComms.h"

SerialComms::SerialComms(int baudRate)
    : _baudRate(baudRate) {}

void SerialComms::begin() {
    Serial.begin(_baudRate);
}

MotorCommand SerialComms::read() {
    MotorCommand cmd = {0.0f, 0.0f, 90, false, false, false, false, false, false};

    if (!Serial.available()) {
        return cmd;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "HOME") {
        cmd.home = true;
        return cmd;
    }

    if (input == "ROTATE") {
        cmd.rotate = true;
        return cmd;
    }

    if (input == "ROTATE_HOME") {
        cmd.rotate_home = true;
        return cmd;
    }

    if (input == "PICK") {
        cmd.pick = true;
        return cmd;
    }

    if (input == "DROP") {
        cmd.drop = true;
        return cmd;
    }

    int comma1 = input.indexOf(',');
    if (comma1 == -1) {
        return cmd;     // malformed, no comma found
    }

    int comma2 = input.indexOf(',', comma1 + 1);

    cmd.angle1 = input.substring(0, comma1).toFloat();
    if (comma2 == -1) {
        // two-value format (backwards compat)
        cmd.angle2 = input.substring(comma1 + 1).toFloat();
    } else {
        cmd.angle2      = input.substring(comma1 + 1, comma2).toFloat();
        cmd.servo_angle = input.substring(comma2 + 1).toInt();
    }
    cmd.valid = true;

    return cmd;
}
