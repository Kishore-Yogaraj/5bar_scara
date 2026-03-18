#include <Arduino.h>
#include "config.h"
#include "motor/DCMotor.h"
#include "encoder/Encoder.h"
#include "pid/PIDController.h"

//Motor 1 
Encoder       enc1(M1_ENC_A, M1_ENC_B, M1_PCNT_UNIT);
PIDController pid1(0.4f, 0.0f, 0.005f, 5000.0f, 10.0f);
DCMotor       motor1(M1_RPWM_PIN, M1_LPWM_PIN, M1_RPWM_CH, M1_LPWM_CH, enc1, pid1);

//Motor 2
Encoder       enc2(M2_ENC_A, M2_ENC_B, M2_PCNT_UNIT);
PIDController pid2(0.4f, 0.0f, 0.005f, 5000.0f, 10.0f);
DCMotor       motor2(M2_RPWM_PIN, M2_LPWM_PIN, M2_RPWM_CH, M2_LPWM_CH, enc2, pid2);


void setup() {
    Serial.begin(115200);
    motor1.begin();
    motor1.setTargetDegrees(360.0f);
    motor2.begin();
    motor2.setTargetDegrees(360.0f);
    Serial.println("Motors ready.");
}

void loop() {
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
    delay(10);
}