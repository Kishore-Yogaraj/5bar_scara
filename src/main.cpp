#include <Arduino.h>
#include "config.h"
#include "motor/DCMotor.h"
#include "encoder/Encoder.h"
#include "pid/PIDController.h"
#include "comms/SerialComms.h"
#include "servo/MyServo.h"
#include "stepper/StepperMotor.h"
#include "gripper/Gripper.h"

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

//Gripper Setup
MyServo     gripServo(GRIP_SERVO_PIN, GRIP_SERVO_CH);
MyServo     liftServo(LIFT_SERVO_PIN, LIFT_SERVO_CH);
Gripper     myGripper(gripServo, liftServo);

//Stepper Setup
StepperMotor stepper(SM_step, SM_dir, STEPS_PER_REV);


bool          moving           = false;
int           pendingServoAngle = 0;
unsigned long lastEncPrint     = 0;

void setup() {
    comms.begin();
    motor1.begin();
    motor2.begin();
    rotatemotor.begin();
    rotatemotor.writeAngle(0);   // start at neutral
    myGripper.begin(GRIP_OPEN_ANGLE, GRIP_CLOSED_ANGLE, LIFT_UP_ANGLE, LIFT_DOWN_ANGLE);
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
    moving       = true;
    lastEncPrint = 0;   // force first periodic print immediately
  }
  else if (cmd.rotate) {
    stepper.setDirection(false); //CW — to pick zone
    stepper.moveAngleSinusoidal(45, SM_MIN_DELAY, SM_MAX_DELAY);
    Serial.println("DONE");
  }
  else if (cmd.rotate_home) {
    stepper.setDirection(true);  //CCW — back to drop zone
    stepper.moveAngleSinusoidal(45, SM_MIN_DELAY, SM_MAX_DELAY);
    Serial.println("DONE");
  }
  else if (cmd.pick) {
    myGripper.pickSequence();
    while (myGripper.isRunningPick()) {
      myGripper.update();
      motor1.update();
      motor2.update();
      delay(10);
    }
    rotatemotor.writeAngle(0);
    unsigned long pt = millis();
    while (millis() - pt < 500) { myGripper.update(); motor1.update(); motor2.update(); delay(10); }
    Serial.println("DONE");
  }
  else if (cmd.drop) {
    myGripper.dropSequence();
    while (myGripper.isRunningDrop()) {
      myGripper.update();
      motor1.update();
      motor2.update();
      delay(10);
    }
    Serial.println("DONE");
  }

    motor1.update();
    // Serial.print("Target: ");
    // Serial.print(motor1.getTargetTicks());
    // Serial.print(" M1: ");
    // Serial.println(enc1.getPosition());

    motor2.update();
    // Serial.print("Target: ");
    // Serial.print(motor2.getTargetTicks());
    // Serial.print(" M2: ");
    // Serial.println(enc2.getPosition());

    myGripper.update();

    if (moving) {
        unsigned long now = millis();
        if (now - lastEncPrint >= 500) {
            lastEncPrint = now;
            Serial.printf("ENC t1=%.0f p1=%d t2=%.0f p2=%d\n",
                motor1.getTargetTicks(), (int)enc1.getPosition(),
                motor2.getTargetTicks(), (int)enc2.getPosition());
        }
        float err1 = abs(motor1.getTargetTicks() - (float)enc1.getPosition());
        float err2 = abs(motor2.getTargetTicks() - (float)enc2.getPosition());
        if (err1 <= 10.0f && err2 <= 10.0f) {
            moving = false;
            // Hold position while PID settles
            unsigned long t = millis();
            while (millis() - t < 1000) { motor1.update(); motor2.update(); delay(10); }
            // Rotate gripper to object angle; servo stays here until PICK rotates it back
            rotatemotor.writeAngle(pendingServoAngle);
            t = millis();
            while (millis() - t < 500) { motor1.update(); motor2.update(); delay(10); }
            Serial.printf("ENC t1=%.0f p1=%d t2=%.0f p2=%d\n",
                motor1.getTargetTicks(), (int)enc1.getPosition(),
                motor2.getTargetTicks(), (int)enc2.getPosition());
            Serial.println("DONE");
        }
    }

    delay(10);

  }
