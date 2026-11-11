/**
 * @file buzzer_driver.c
 * @brief Implementación LEDC y FreeRTOS para avisos acústicos en GPIO 38.
 */

#include "buzzer_driver.h"
#include "pinout.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUZZER";

#define BUZZER_LEDC_TIMER       LEDC_TIMER_1
#define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES    LEDC_TIMER_8_BIT

static TaskHandle_t s_buzzer_task_handle = NULL;

static void buzzer_hw_tone(uint16_t freq_hz, uint8_t duty_pct)
{
    if (freq_hz == 0 || duty_pct == 0) {
        ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
        return;
    }

    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    uint32_t duty = (255 * duty_pct) / 100;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

esp_err_t buzzer_init(void)
{
    ESP_LOGI(TAG, "Inicializando Buzzer en GPIO %d...", PIN_NUM_BUZZER);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .timer_num        = BUZZER_LEDC_TIMER,
        .duty_resolution  = BUZZER_LEDC_DUTY_RES,
        .freq_hz          = 2400,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_NUM_BUZZER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Máxima capacidad de corriente en GPIO 38 para excitar el buzzer pasivo (40 mA)
    gpio_set_drive_capability(PIN_NUM_BUZZER, GPIO_DRIVE_CAP_3);

    buzzer_stop();
    return ESP_OK;
}

void buzzer_stop(void)
{
    buzzer_hw_tone(0, 0);
}

void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    buzzer_hw_tone(freq_hz, 50);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_hw_tone(0, 0);
}

typedef enum {
    BUZZER_PAT_NONE = 0,
    BUZZER_PAT_JOB_COMPLETE,
    BUZZER_PAT_ALERT
} buzzer_pattern_t;

static void buzzer_pattern_task(void *pvParameters)
{
    buzzer_pattern_t pattern = (buzzer_pattern_t)(uintptr_t)pvParameters;
    ESP_LOGI(TAG, "Reproduciendo patron acustico en GPIO %d (patron %d)...", PIN_NUM_BUZZER, (int)pattern);

    if (pattern == BUZZER_PAT_JOB_COMPLETE) {
        // Melodía ascendente de fin de trabajo: 3 beeps claros a resonancia acústica (50% duty)
        buzzer_hw_tone(2048, 50);
        vTaskDelay(pdMS_TO_TICKS(150));
        buzzer_hw_tone(0, 0);
        vTaskDelay(pdMS_TO_TICKS(80));

        buzzer_hw_tone(2400, 50);
        vTaskDelay(pdMS_TO_TICKS(150));
        buzzer_hw_tone(0, 0);
        vTaskDelay(pdMS_TO_TICKS(80));

        buzzer_hw_tone(2700, 50);
        vTaskDelay(pdMS_TO_TICKS(300));
        buzzer_hw_tone(0, 0);
    } else if (pattern == BUZZER_PAT_ALERT) {
        // Alerta de seguridad intermitente rápida
        for (int i = 0; i < 5; i++) {
            buzzer_hw_tone(3000, 50);
            vTaskDelay(pdMS_TO_TICKS(80));
            buzzer_hw_tone(0, 0);
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }

    s_buzzer_task_handle = NULL;
    vTaskDelete(NULL);
}

void buzzer_play_job_complete(void)
{
    if (s_buzzer_task_handle != NULL) return; // Ya hay un patrón sonando
    ESP_LOGI(TAG, "Solicitado aviso acustico de corte finalizado (BUZZER_PAT_JOB_COMPLETE).");
    xTaskCreate(buzzer_pattern_task, "buzzer_pat", 2048, 
                (void *)(uintptr_t)BUZZER_PAT_JOB_COMPLETE, 5, &s_buzzer_task_handle);
}

void buzzer_play_alert(void)
{
    if (s_buzzer_task_handle != NULL) return;
    xTaskCreate(buzzer_pattern_task, "buzzer_pat", 2048, 
                (void *)(uintptr_t)BUZZER_PAT_ALERT, 5, &s_buzzer_task_handle);
}

