#include <Arduino.h>
#include "config.h"
#include "motor/DCMotor.h"
#include "encoder/Encoder.h"
#include "pid/PIDController.h"
#include "comms/SerialComms.h"
#include "servo/MyServo.h"
#include "stepper/StepperMotor.h"

//Motor 1
Encoder       enc1(M1_ENC_A, M1_ENC_B, M1_PCNT_UNIT);
PIDController pid1(6.0f, 0.001f, 0.16f, 5000.0f, 10.0f);
DCMotor       motor1(M1_RPWM_PIN, M1_LPWM_PIN, M1_RPWM_CH, M1_LPWM_CH, enc1, pid1);

//Motor 2
Encoder       enc2(M2_ENC_A, M2_ENC_B, M2_PCNT_UNIT);
PIDController pid2(5.5f, 0.001f, 0.16f, 5000.0f, 10.0f);
DCMotor       motor2(M2_RPWM_PIN, M2_LPWM_PIN, M2_RPWM_CH, M2_LPWM_CH, enc2, pid2);

//Comms Setup
SerialComms comms(115200);

//Servo Setup
MyServo     rotatemotor(19, 4);

//Stepper Setup
StepperMotor stepper(SM_step, SM_dir, STEPS_PER_REV);


bool moving          = false;
int  pendingServoAngle = 0;

void setup() {
    comms.begin();
    motor1.begin();
    motor2.begin();
    rotatemotor.begin();
    rotatemotor.writeAngle(0);   // start at neutral
    stepper.begin();
    Serial.println("Motors ready.");
}

void loop() {
  MotorCommand cmd = comms.read();

  if (cmd.home) {
    enc1.reset();
    enc2.reset();
    motor1.setTargetDegrees(0.0f);
    motor2.setTargetDegrees(0.0f);
    moving = false;
    Serial.println("HOME: encoders reset to 0.");
  }
  else if (cmd.valid) {
    motor1.setTargetDegrees(cmd.angle1);
    motor2.setTargetDegrees(cmd.angle2);
    pendingServoAngle = cmd.servo_angle;
    moving = true;
  }
  else if (cmd.rotate) {
    stepper.setDirection(false);   // true = CCW — flip to false if rotation direction is wrong
    stepper.moveAngleSinusoidal(45, SM_MIN_DELAY, SM_MAX_DELAY);
    Serial.println("DONE");
  }

    motor1.update();
    Serial.print("Target: ");
    Serial.print(motor1.getTargetTicks());
    Serial.print(" M1: ");
    Serial.println(enc1.getPosition());

    motor2.update();
    Serial.print("Target: ");
    Serial.print(motor2.getTargetTicks());
    Serial.print(" M2: ");
    Serial.println(enc2.getPosition());

    if (moving) {
        float err1 = abs(motor1.getTargetTicks() - (float)enc1.getPosition());
        float err2 = abs(motor2.getTargetTicks() - (float)enc2.getPosition());
        if (err1 <= 10.0f && err2 <= 10.0f) {
            moving = false;
            // Hold position while waiting (keep PID running to prevent drift)
            unsigned long t = millis();
            while (millis() - t < 1000) { motor1.update(); motor2.update(); delay(10); }
            rotatemotor.writeAngle(pendingServoAngle + 10);
            t = millis();
            while (millis() - t < 1000) { motor1.update(); motor2.update(); delay(10); }
            rotatemotor.writeAngle(0);
            Serial.println("DONE");
        }
    }

    delay(10);

  }
