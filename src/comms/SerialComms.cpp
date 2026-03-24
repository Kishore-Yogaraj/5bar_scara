#include "SerialComms.h"

SerialComms::SerialComms(int baudRate)
    : _baudRate(baudRate) {}

void SerialComms::begin() {
    Serial.begin(_baudRate);
}

MotorCommand SerialComms::read() {
    MotorCommand cmd = {0};

    if (!Serial.available()) {
        return cmd;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    // Debug (optional but useful)
    Serial.print("RAW: ");
    Serial.println(input);

    // HOME
    if (input == "HOME") {
        cmd.home = true;
        return cmd;
    }

    // ANGLE command: A,angle1,angle2
    if (input.startsWith("A,")) {
        const char* str = input.c_str() + 2;

        int parsed = sscanf(str, "%f,%f",
                            &cmd.angle1, &cmd.angle2);

        if (parsed == 2) {
            cmd.valid = true;
        }
    }

    // PID command: P,Kp1,Ki1,Kd1,Kp2,Ki2,Kd2
    else if (input.startsWith("P,")) {
        const char* str = input.c_str() + 2;

        int parsed = sscanf(str, "%f,%f,%f,%f,%f,%f",
                            &cmd.Kp1, &cmd.Ki1, &cmd.Kd1,
                            &cmd.Kp2, &cmd.Ki2, &cmd.Kd2);

        Serial.print("Parsed: ");
        Serial.println(parsed);  // DEBUG

        if (parsed == 6) {
            cmd.pidUpdate = true;
        }
    }

    return cmd;
}