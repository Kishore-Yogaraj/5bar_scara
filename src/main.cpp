#include <Arduino.h>
#include "driver/pcnt.h"

//Pin Definitions
#define ENC_A    25
#define ENC_B    26
#define RPWM_PIN 27
#define LPWM_PIN 14

//PWM Config
#define PWM_FREQ        2000
#define PWM_RESOLUTION  8
#define RPWM_CH         0
#define LPWM_CH         1

//Encoder config
#define CPR 4192.0f

//Target
const float TARGET_DEGREES = 90.0f;

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

  //Set to 0 so motors don't move on startup
  ledcWrite(RPWM_CH, 0);
  ledcWrite(LPWM_CH, 0);


  //Set up encoder - setting encoder to 0
  setupEncoder();

  Serial.println("PWM ready + Encoder ready");
  Serial.println("Raw Encoder Count:");
}


void loop(){
  Serial.println(getPosition());
  delay(1000);
}