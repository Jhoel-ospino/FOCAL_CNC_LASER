/**
 * @file tmc2209.c
 * @brief Implementación del driver Trinamic TMC2209 en modo Standalone (STEP/DIR + Enable común).
 */

#include "tmc2209.h"
#include "pinout.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMC2209";

static bool s_drivers_enabled = true;

esp_err_t tmc2209_init(void)
{
    ESP_LOGI(TAG, "Inicializando drivers TMC2209 en modo Standalone (STEP/DIR)...");

    // Configurar el pin Enable común (GPIO 3)
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_MOTORS_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_conf);

    // Habilitar etapas de potencia (Activo en LOW para TMC2209)
    gpio_set_level(PIN_NUM_MOTORS_EN, 0);
    s_drivers_enabled = true;

    ESP_LOGI(TAG, "Drivers TMC2209 Standalone listos (MOTORS_EN en GPIO %d habilitado en LOW).", PIN_NUM_MOTORS_EN);
    return ESP_OK;
}

esp_err_t tmc2209_set_current(uint8_t axis, uint16_t current_ma)
{
    // En modo Standalone sin UART, la corriente se fija por el potenciómetro (Vref) de cada módulo StepStick
    ESP_LOGD(TAG, "Ajuste de corriente para eje %d: %d mA (Modo Standalone: calibrar Vref en protoboard).", axis, current_ma);
    return ESP_OK;
}

esp_err_t tmc2209_set_microsteps(uint8_t axis, uint16_t microsteps)
{
    // En modo Standalone, los micropasos se fijan por el estado de MS1/MS2 en la protoboard
    ESP_LOGD(TAG, "Ajuste de micropasos para eje %d: %d (Modo Standalone: MS1/MS2 en protoboard).", axis, microsteps);
    return ESP_OK;
}

esp_err_t tmc2209_enable_stealthchop(uint8_t axis, bool enable)
{
    // En modo Standalone, StealthChop2 viene activo de fábrica en TMC2209 con microPlyer 256x nativo
    return ESP_OK;
}

esp_err_t tmc2209_enable_drivers(bool enable)
{
    s_drivers_enabled = enable;
    gpio_set_level(PIN_NUM_MOTORS_EN, enable ? 0 : 1);
    ESP_LOGI(TAG, "TMC2209 drivers %s (GPIO %d = %d)", enable ? "habilitados" : "deshabilitados", 
             PIN_NUM_MOTORS_EN, enable ? 0 : 1);
    return ESP_OK;
}

bool tmc2209_is_enabled(void)
{
    return s_drivers_enabled;
}
