#include <Arduino.h>
#include "driver/pcnt.h"

//Pin Definitions
#define ENC_A    25
#define ENC_B    26
#define RPWM_PIN 27
#define LPWM_PIN 14

//PWM Config
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define RPWM_CH         0 //Controls CW when set to high
#define LPWM_CH         1 //Controls CCW when set to high

//Encoder config
#define CPR 4192.0f

//PID state
float target_degrees = 360.0f; //set in degrees
float target_ticks = 0.0f; //Used for PID Control Loop
float prev_error = 0.0f;
float integral = 0.0f;
unsigned long prevTime = 0;


//PID gains
float kp = 0.4f;
float ki = 0.0f;
float kd = 0.005f;

//PWM cap
#define PWM_MAX 150


// //Target
// const float TARGET_DEGREES = 360.0f;

void setupEncoder() {
    pcnt_config_t pcnt_config = {};

    pcnt_config.pulse_gpio_num = ENC_A;
    pcnt_config.ctrl_gpio_num  = ENC_B;
    pcnt_config.unit           = PCNT_UNIT_0;
    pcnt_config.channel        = PCNT_CHANNEL_0;

    // on ENC_A rising edge: if ENC_B is LOW count up, if HIGH count down
    pcnt_config.pos_mode   = PCNT_COUNT_INC;
    pcnt_config.neg_mode   = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

    // hardware limits (int16 max range)
    pcnt_config.counter_h_lim =  32767;
    pcnt_config.counter_l_lim = -32768;

    pcnt_unit_config(&pcnt_config);

    // filter out glitches shorter than 10 cycles: 1 cycle = 1/80 000 00 of a second esp runs on 80MHz
    // if we read a pulse that lasts less than 125 nanoseconds then ignore it
    pcnt_set_filter_value(PCNT_UNIT_0, 10);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

int getPosition() {
    int16_t count;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return count;
}

void update(){
  //Change in time calculation
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime)/1000.0f; //change in time in seconds

  if (dt <= 0.001f){
    return;
  }

  prevTime = currentTime;

  //Calculating error
  float position = getPosition();
  float error = target_ticks - position;
  if (abs(error) < 10) {
    integral = 0;
    ledcWrite(RPWM_CH, 0);
    ledcWrite(LPWM_CH, 0);
    return;
  }

  //Integral Calculation
  integral += error * dt;
  integral = constrain(integral, -5000, 5000);

  //Derivative calculation
  float derivative = (error - prev_error)/dt;

  float control_signal = kp * error + ki * integral + kd * derivative;
  prev_error = error;

  //Converting to PWM------
  int pwm = (int)(constrain(abs(control_signal), 0, PWM_MAX));

  //Positive control signal is CW
  if (control_signal >= 0){
    ledcWrite(RPWM_CH, pwm);
    ledcWrite(LPWM_CH, 0);
  }
  else{
    ledcWrite(RPWM_CH, 0);
    ledcWrite(LPWM_CH, pwm);
  }
}

void setup() {
  Serial.begin(115200);
  
  // //PWM setup
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  
  ledcSetup(RPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, RPWM_CH);
  ledcSetup(LPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LPWM_PIN, LPWM_CH);

  //Set up encoder - setting encoder to 0
  setupEncoder();

  prevTime = millis();

  target_ticks = (target_degrees/360.0f) * CPR;

  Serial.println("PWM ready + Encoder ready");
}


void loop(){
  update();

  Serial.print("Target: ");
  Serial.print(target_ticks);
  Serial.print("  Position: ");
  Serial.println(getPosition());

  delay(10);
}