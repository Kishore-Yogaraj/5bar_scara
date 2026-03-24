#include <Arduino.h>
#include "config.h"
#include "motor/DCMotor.h"
#include "StepperMotor/StepperMotor.h"
#include "encoder/Encoder.h"
#include "pid/PIDController.h"
#include "comms/SerialComms.h"
//Stepper motor
StepperMotor stepper(SM_step, SM_dir, STEPS_PER_REV);

//Motor 1 
Encoder       enc1(M1_ENC_A, M1_ENC_B, M1_PCNT_UNIT);
PIDController pid1(2.2f, 0.000f, 0.10f, 5000.0f, 10.0f);
DCMotor       motor1(M1_RPWM_PIN, M1_LPWM_PIN, M1_RPWM_CH, M1_LPWM_CH, enc1, pid1);

//Motor 2
Encoder       enc2(M2_ENC_A, M2_ENC_B, M2_PCNT_UNIT);
PIDController pid2(2.2f, 0.000f, 0.10f, 5000.0f, 10.0f);
DCMotor       motor2(M2_RPWM_PIN, M2_LPWM_PIN, M2_RPWM_CH, M2_LPWM_CH, enc2, pid2);

//Comms Setup
SerialComms comms(115200);



void setup() {
    comms.begin();
    motor1.begin();
    motor2.begin();
    Serial.println("Motors ready.");
}

void loop() {
  MotorCommand cmd = comms.read();

  if (cmd.home) {
    enc1.reset();
    enc2.reset();
    motor1.setTargetDegrees(0.0f);
    motor2.setTargetDegrees(0.0f);
    Serial.println("HOME: encoders reset to 0.");
  }
  if (cmd.valid) {
    motor1.setTargetDegrees(cmd.angle1);
    motor2.setTargetDegrees(cmd.angle2);

  }
  // PID update command
  // if (cmd.pidUpdate) {
  //       pid1.setGains(cmd.Kp1, cmd.Ki1, cmd.Kd1);
  //       pid2.setGains(cmd.Kp2, cmd.Ki2, cmd.Kd2);
  //       Serial.print("PID Updated: M1(");
  //       Serial.print(cmd.Kp1); Serial.print(", "); Serial.print(cmd.Ki1); Serial.print(", "); Serial.print(cmd.Kd1);
  //       Serial.print(") M2(");
  //       Serial.print(cmd.Kp2); Serial.print(", "); Serial.print(cmd.Ki2); Serial.print(", "); Serial.print(cmd.Kd2);
  //       Serial.println(")");
  //   }
    motor1.setPID(6.0,0.01,0.16);
    motor2.setPID(5.5,0.01,0.16);


    motor1.update();
    Serial.print("Target1: ");
    Serial.print(motor1.getTargetTicks());
    Serial.print(" M1: "); 
    Serial.println(enc1.getPosition());
    
    motor2.update();

    Serial.print("Target2: ");
    Serial.print(motor2.getTargetTicks());
    Serial.print(" M2: "); 
    Serial.println(enc2.getPosition());
    delay(10);

    
    
    
    
}