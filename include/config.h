#pragma once

//Motor 1 
#define M1_ENC_A     25
#define M1_ENC_B     26
#define M1_RPWM_PIN  27
#define M1_LPWM_PIN  14
#define M1_RPWM_CH   0
#define M1_LPWM_CH   1
#define M1_PCNT_UNIT PCNT_UNIT_0

//Motor 2
#define M2_ENC_A     34
#define M2_ENC_B     35
#define M2_RPWM_PIN  32
#define M2_LPWM_PIN  33
#define M2_RPWM_CH   2
#define M2_LPWM_CH   3
#define M2_PCNT_UNIT PCNT_UNIT_1

//Shared hardware config
#define PWM_FREQ       20000
#define PWM_RESOLUTION 8
#define PWM_MAX        150
#define CPR            4192.0f