#pragma once
#include <stdint.h>

#define FAN_PWM_GPIO     13// on esp32 ethernet kit
#define FAN_PWM_CHANNEL  LEDC_CHANNEL_0
#define FAN_PWM_TIMER    LEDC_TIMER_0
#define FAN_PWM_FREQ_HZ  500
#define FAN_PWM_RES      LEDC_TIMER_10_BIT

extern float T_set;
extern bool manual_mode;

// Initialize PWM fan and controller
void fan_control_init(void);

// Compute PI output and update fan PWM
void fan_control_update(float temperature_C);

// Get last computed fan percentage
float fan_get_last_command(void);

// Test function for pwm fan
void fan_control_force(float duty_percent);
