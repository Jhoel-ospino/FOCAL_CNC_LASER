/**
 * @file laser_driver.c
 * @brief Implementación LEDC PWM de alta frecuencia para Tree Laser 20W en GPIO 18.
 */

#include "laser_driver.h"
#include "pinout.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LASER_20W";

#define LASER_LEDC_TIMER       LEDC_TIMER_0
#define LASER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LASER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define LASER_LEDC_DUTY_RES    LEDC_TIMER_10_BIT // 0 - 1023
#define LASER_LEDC_FREQUENCY   5000              // 5 kHz para corte limpio sin banding

static uint16_t s_current_s = 0;
static bool s_enabled = true; // Habilitado por software por defecto (duty inicial = 0)

esp_err_t laser_init(void)
{
    ESP_LOGI(TAG, "Configurando PWM para Tree Laser 20W en GPIO %d (5 kHz, 10-bit)...", PIN_NUM_LASER_PWM);

    // Configurar Timer LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LASER_LEDC_MODE,
        .timer_num        = LASER_LEDC_TIMER,
        .duty_resolution  = LASER_LEDC_DUTY_RES,
        .freq_hz          = LASER_LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configurar Canal LEDC
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LASER_LEDC_MODE,
        .channel        = LASER_LEDC_CHANNEL,
        .timer_sel      = LASER_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_NUM_LASER_PWM,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    s_current_s = 0;
    s_enabled = true;

    ESP_LOGI(TAG, "Módulo láser listo (PWM GPIO %d, Duty inicial = 0).", PIN_NUM_LASER_PWM);
    return ESP_OK;
}

void laser_set_power(uint16_t s_value)
{
    if (s_value > LASER_MAX_S_VAL) s_value = LASER_MAX_S_VAL;
    s_current_s = s_value;

    if (!s_enabled || s_value == 0) {
        ledc_set_duty(LASER_LEDC_MODE, LASER_LEDC_CHANNEL, 0);
        ledc_update_duty(LASER_LEDC_MODE, LASER_LEDC_CHANNEL);
        return;
    }

    // Escalar 0..1000 a 0..1023
    uint32_t duty = (uint32_t)((s_value * 1023) / LASER_MAX_S_VAL);
    ledc_set_duty(LASER_LEDC_MODE, LASER_LEDC_CHANNEL, duty);
    ledc_update_duty(LASER_LEDC_MODE, LASER_LEDC_CHANNEL);
}

void laser_set_enable(bool enable)
{
    s_enabled = enable;
    if (!enable) {
        laser_set_power(0);
    }
}

uint16_t laser_get_power(void)
{
    return s_current_s;
}

uint8_t laser_get_percent(void)
{
    return (uint8_t)((s_current_s * 100) / LASER_MAX_S_VAL);
}
