/**
 * @file stepper_timer.c
 * @brief High-precision Real-Time Motion Execution Engine for ESP32-S3 Core 1
 *        Implements deterministic Bresenham step pulse generation with real-time
 *        trapezoidal velocity profiling and lockstep Dual-Y synchronization.
 */

#include <math.h>
#include "stepper_timer.h"
#include "planner.h"
#include "pinout.h"
#include "laser_driver.h"
#include "buzzer_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STEPPER_CORE1";

#ifndef CONFIG_CNC_INVERT_Y2_DIR
#define CONFIG_CNC_INVERT_Y2_DIR 0
#endif

static cnc_state_t s_cnc_state = CNC_STATE_IDLE;
static bool s_motors_enabled = true;
static TaskHandle_t s_motion_task_handle = NULL;

void stepper_timer_enable_motors(bool enable)
{
    s_motors_enabled = enable;
    gpio_set_level(PIN_MOTORS_EN, enable ? 0 : 1);
    ESP_LOGI(TAG, "Etapa de potencia: %s (GPIO %d = %d)", 
             enable ? "HABILITADA (LOW)" : "DESHABILITADA (HIGH)", PIN_MOTORS_EN, enable ? 0 : 1);
}

bool stepper_timer_is_motors_enabled(void)
{
    return s_motors_enabled;
}

void stepper_timer_start(void)
{
    s_cnc_state = CNC_STATE_RUN;
    if (s_motion_task_handle) {
        xTaskNotifyGive(s_motion_task_handle);
    }
}

void stepper_timer_stop(void)
{
    s_cnc_state = CNC_STATE_HOLD;
    laser_set_power(0);
}

cnc_state_t stepper_timer_get_state(void)
{
    return s_cnc_state;
}

void stepper_timer_set_state(cnc_state_t state)
{
    s_cnc_state = state;
    if (state != CNC_STATE_RUN) {
        laser_set_power(0);
    } else {
        if (s_motion_task_handle) {
            xTaskNotifyGive(s_motion_task_handle);
        }
    }
}

/**
 * @brief Ejecuta un bloque cinemático completo con rampa trapezoidal de velocidad.
 */
static void stepper_execute_block(plan_block_t *blk)
{
    if (!blk || blk->step_event_count == 0 || blk->millimeters < 0.001f) {
        return;
    }

    uint32_t total_steps = blk->step_event_count;
    float dist = blk->millimeters;

    // 1. Configurar pines de dirección
    gpio_set_level(PIN_DIR_X, blk->dir_x);
    gpio_set_level(PIN_DIR_Y1, blk->dir_y);
#if CONFIG_CNC_INVERT_Y2_DIR
    gpio_set_level(PIN_DIR_Y2, !blk->dir_y);
#else
    gpio_set_level(PIN_DIR_Y2, blk->dir_y);
#endif
    esp_rom_delay_us(10); // Tiempo de establecimiento de dirección para TMC2209

    // 2. Parámetros de la rampa trapezoidal
    float v_entry = (blk->entry_speed > 2.0f) ? blk->entry_speed : 2.0f;
    float v_nominal = (blk->nominal_speed > 2.0f) ? blk->nominal_speed : 2.0f;
    float v_exit = (blk->exit_speed > 2.0f) ? blk->exit_speed : 2.0f;
    float accel = (blk->acceleration > 100.0f) ? blk->acceleration : 800.0f;

    // Distancias de aceleración y desaceleración
    float d_accel = 0.0f;
    if (v_nominal > v_entry) {
        d_accel = (v_nominal * v_nominal - v_entry * v_entry) / (2.0f * accel);
    }
    float d_decel = 0.0f;
    if (v_nominal > v_exit) {
        d_decel = (v_nominal * v_nominal - v_exit * v_exit) / (2.0f * accel);
    }

    // Si el bloque es muy corto para alcanzar velocidad nominal (perfil triangular)
    if (d_accel + d_decel > dist) {
        float v_peak_sq = (2.0f * accel * dist + v_entry * v_entry + v_exit * v_exit) / 2.0f;
        if (v_peak_sq > 4.0f) {
            v_nominal = sqrtf(v_peak_sq);
            d_accel = (v_nominal * v_nominal - v_entry * v_entry) / (2.0f * accel);
            d_decel = dist - d_accel;
        } else {
            v_nominal = 2.0f;
            d_accel = 0.0f;
            d_decel = 0.0f;
        }
    }

    uint32_t counter_x = total_steps / 2;
    uint32_t counter_y = total_steps / 2;

    // 3. Bucle de emisión determinista paso a paso
    for (uint32_t step = 0; step < total_steps; step++) {
        // Verificar si se solicitó pausa o parada de emergencia
        if (s_cnc_state == CNC_STATE_HOLD || s_cnc_state == CNC_STATE_IDLE) {
            laser_set_power(0);
            return;
        }

        // Posición actual a lo largo de la trayectoria
        float s = (float)step * dist / (float)total_steps;

        // Calcular velocidad instantánea v(s)
        float v = v_nominal;
        if (s < d_accel && d_accel > 0.001f) {
            float v_sq = v_entry * v_entry + 2.0f * accel * s;
            v = (v_sq > 4.0f) ? sqrtf(v_sq) : 2.0f;
        } else if (s > (dist - d_decel) && d_decel > 0.001f) {
            float s_decel = dist - s;
            float v_sq = v_exit * v_exit + 2.0f * accel * s_decel;
            v = (v_sq > 4.0f) ? sqrtf(v_sq) : 2.0f;
        }

        if (v < 2.0f) v = 2.0f;
        if (v > 50.0f) v = 50.0f; // 3000 mm/min límite seguro para husillo T8

        // Modulación dinámica de potencia láser M4
        if (blk->dynamic_laser) {
            float ratio = (blk->nominal_speed > 0.0f) ? (v / blk->nominal_speed) : 0.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            laser_set_power((uint16_t)(blk->laser_power * ratio));
        } else {
            laser_set_power(blk->laser_power);
        }

        // Periodo entre pasos (en microsegundos)
        // R = (total_steps / dist) * v (pasos/segundo)
        float step_rate = (dist > 0.0001f) ? ((float)total_steps / dist) * v : 800.0f;
        uint32_t delay_us = (uint32_t)(1000000.0f / step_rate);
        if (delay_us < 40) delay_us = 40;     // Max 25 kHz
        if (delay_us > 2500) delay_us = 2500; // Min 400 Hz

        // Algoritmo DDA Bresenham
        bool pulse_x = false;
        bool pulse_y = false;

        counter_x += blk->steps_x;
        if (counter_x >= total_steps) {
            counter_x -= total_steps;
            pulse_x = true;
        }

        counter_y += blk->steps_y;
        if (counter_y >= total_steps) {
            counter_y -= total_steps;
            pulse_y = true;
        }

        // Flanco de subida simultáneo (Dual-Y estricto)
        if (pulse_x) gpio_set_level(PIN_STEP_X, 1);
        if (pulse_y) {
            gpio_set_level(PIN_STEP_Y1, 1);
            gpio_set_level(PIN_STEP_Y2, 1);
        }

        // Ancho de pulso TMC2209 (4 microsegundos)
        esp_rom_delay_us(4);

        // Flanco de bajada
        if (pulse_x) gpio_set_level(PIN_STEP_X, 0);
        if (pulse_y) {
            gpio_set_level(PIN_STEP_Y1, 0);
            gpio_set_level(PIN_STEP_Y2, 0);
        }

        // Espera del periodo de paso restante
        if (delay_us > 4) {
            esp_rom_delay_us(delay_us - 4);
        }
    }
}

/**
 * @brief Tarea de Tiempo Real ejecutada exclusivamente en Core 1.
 */
static void task_motion_realtime(void *pvParameters)
{
    ESP_LOGI(TAG, "Tarea de Control de Movimiento iniciada en Core %d (Prioridad %d).", 
             xPortGetCoreID(), (int)uxTaskPriorityGet(NULL));

    // TEST DE DIAGNOSTICO EN ARRANQUE:
    // Emite una ráfaga suave de 150 pulsos a 350 Hz en los 3 motores (X, Y1, Y2).
    // Si la fuente de 12V y los cables están conectados, los motores emitirán un zumbido
    // audible y claro de 0.4 segundos, confirmando vida física.
    ESP_LOGI(TAG, ">>> INICIANDO TEST DE DIAGNOSTICO FISICO DE MOTORES (0.4s @ 350Hz)...");
    stepper_timer_enable_motors(true);
    for (int i = 0; i < 150; i++) {
        gpio_set_level(PIN_STEP_X, 1);
        gpio_set_level(PIN_STEP_Y1, 1);
        gpio_set_level(PIN_STEP_Y2, 1);
        esp_rom_delay_us(5);
        gpio_set_level(PIN_STEP_X, 0);
        gpio_set_level(PIN_STEP_Y1, 0);
        gpio_set_level(PIN_STEP_Y2, 0);
        esp_rom_delay_us(2800); // 2.8 ms = ~350 Hz (frecuencia acústica claramente audible)
    }
    ESP_LOGI(TAG, ">>> TEST DE DIAGNOSTICO FISICO DE MOTORES COMPLETADO.");

    while (1) {
        if (s_cnc_state != CNC_STATE_RUN) {
            laser_set_power(0);
            // Dormir esperando notificación de nuevo comando (Cero consumo de CPU en reposo)
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
            continue;
        }

        plan_block_t *block = planner_get_current_block();
        if (!block) {
            // Buffer agotado -> volver a reposo
            s_cnc_state = CNC_STATE_IDLE;
            laser_set_power(0);
            continue;
        }

        // Ejecutar bloque con perfilador trapezoidal
        stepper_execute_block(block);
        planner_discard_current_block();
    }
}

void stepper_timer_init(void)
{
    ESP_LOGI(TAG, "Configurando pines GPIO de motores...");

    // Configurar STEP / DIR / EN GPIOs como salidas de alta velocidad
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_MOTORS_EN) |
                        (1ULL << PIN_STEP_X)  | (1ULL << PIN_DIR_X)  |
                        (1ULL << PIN_STEP_Y1) | (1ULL << PIN_DIR_Y1) |
                        (1ULL << PIN_STEP_Y2) | (1ULL << PIN_DIR_Y2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Habilitar etapas de potencia de inmediato (Activo en LOW)
    stepper_timer_enable_motors(true);

    gpio_set_level(PIN_STEP_X, 0);
    gpio_set_level(PIN_DIR_X, 0);
    gpio_set_level(PIN_STEP_Y1, 0);
    gpio_set_level(PIN_DIR_Y1, 0);
    gpio_set_level(PIN_STEP_Y2, 0);
    gpio_set_level(PIN_DIR_Y2, 0);

    // Crear la tarea cinemática de tiempo real anclada exclusivamente al Core 1
    xTaskCreatePinnedToCore(
        task_motion_realtime,
        "task_motion_rt",
        8192,
        NULL,
        5, // Prioridad 5 (Tiempo Real)
        &s_motion_task_handle,
        1  // CORE 1 EXCLUSIVO
    );

    ESP_LOGI(TAG, "Motor Engine listo (Core 1, X: GPIO 1/2, Y1: GPIO 4/5, Y2: GPIO 6/7, EN: GPIO 3).");
}
