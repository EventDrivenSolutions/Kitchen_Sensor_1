
// imported from mbed project

#include "fan_control.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "FAN_CTRL";

bool manual_mode = false;
float T_set = 21.7f;          // desired temp--- MAY NEED TO CHANGE FOR CLASS DEMONSTRATION


float integrator = 0.0f;      // integral state
static float Ki = 0.5f;              // integral gain
static float Kp = 250.0f;            // proportional gain
static float min_duty_fan = 20.0f; 
static float last_fan_cmd = 0.0f;

static void fan_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing fan PWM on GPIO %d", FAN_PWM_GPIO);

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .timer_num       = FAN_PWM_TIMER,
        .duty_resolution = FAN_PWM_RES,
        .freq_hz         = FAN_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num       = FAN_PWM_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = FAN_PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = FAN_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

static void fan_pwm_set_percent(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;

    last_fan_cmd = duty;

    uint32_t max_duty = (1 << FAN_PWM_RES) - 1;
    uint32_t raw = (uint32_t)((duty / 100.0f) * max_duty);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, FAN_PWM_CHANNEL, raw);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, FAN_PWM_CHANNEL);
}

float compute_fan_command(float temperature_C) {

    // Compute PI effort
    // Determine Error for controller, positive value means too hot.
    float error = temperature_C - T_set;
    
    // Compute integrator
    float next_integrator = integrator + error; // my change in time dt is 1000ms or 1 second
    
    
    float u_raw = Kp * error + Ki * next_integrator;

    // filter u raw to a percetage between 0 and hunna
    float u_filtered = u_raw;
    if (u_filtered < 0.0f) u_filtered = 0.0f;
    if (u_filtered > 100.0f) u_filtered = 100.0f;

    bool saturating_low  = (u_filtered <= 0.0f  && error < 0);
    bool saturating_high = (u_filtered >= 100.0f && error > 0);

    if (!saturating_low && !saturating_high)
        integrator = next_integrator;

    // --- Minimum duty for physical fan startup ---
    if (u_filtered > 0.0f && u_filtered < min_duty_fan)
        u_filtered = min_duty_fan;

    return u_filtered;

    // P Controller -- Does not reach desired temp - gets stuck when too close
    // // Compute Raw controller proportional output.
    // float u_raw = Kp * error;
    // float cmd;
    // // Filter raw P output for fan so it doesnt command negative or over 100 percent blow.
    // if (u_raw < 0.0f)
    //     cmd = 0.0f;
    // else if (u_raw > 100.0f)
    //     cmd = 100.0f;
    // else
    //     cmd = u_raw;

    // // Enforce minimum duty so the fan actually spins
    // if (cmd > 0.0f && cmd < min_duty_fan)
    //     cmd = min_duty_fan;// 30% duty = fan's minimum reliable starting point

    // return cmd;   // percentage, 0–100
}

void fan_control_init(void)
{
    fan_pwm_init();
}

void fan_control_update(float temperature_C)
{
    float cmd = compute_fan_command(temperature_C);
    fan_pwm_set_percent(cmd);
}

float fan_get_last_command(void)
{
    return last_fan_cmd;
}

void fan_control_force(float duty_percent)
{
    ESP_LOGI(TAG, "Force fan duty = %.1f%%", duty_percent);
    fan_pwm_set_percent(duty_percent);
}

