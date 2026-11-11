/**
 * @file gcode_streamer.c
 * @brief Implementación del motor de streaming de archivos G-code
 *        en segundo plano con control de flujo lookahead FreeRTOS.
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "gcode_streamer.h"
#include "planner.h"
#include "stepper_timer.h"
#include "gcode_parser.h"
#include "laser_driver.h"
#include "buzzer_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GCODE_STREAMER";

static TaskHandle_t s_streamer_task = NULL;
static char s_current_filepath[128] = {0};
static volatile bool s_is_running = false;
static volatile bool s_is_paused = false;
static volatile bool s_stop_requested = false;

static volatile float s_last_pct = 0.0f;
static volatile uint32_t s_last_executed = 0;
static volatile uint32_t s_total_lines = 0;
static volatile uint32_t s_last_elapsed_sec = 0;

static gcode_streamer_progress_cb_t s_progress_cb = NULL;
static gcode_streamer_complete_cb_t s_complete_cb = NULL;

uint32_t gcode_streamer_count_lines(const char *filepath)
{
    if (!filepath || strlen(filepath) == 0) return 0;

    char path[160];
    if (strncmp(filepath, "/sdcard/", 8) == 0) {
        snprintf(path, sizeof(path), "%s", filepath);
    } else if (strncmp(filepath, "/flash/", 7) == 0) {
        snprintf(path, sizeof(path), "/sdcard/%s", filepath + 7);
    } else {
        snprintf(path, sizeof(path), "/sdcard/%s", filepath);
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        f = fopen(filepath, "r");
    }
    if (!f) return 0;

    uint32_t count = 0;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (isspace((unsigned char)*p)) p++;
        if (*p != '\0' && *p != ';' && *p != '(') {
            count++;
        }
    }
    fclose(f);
    return count;
}

static void task_streamer(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando streaming de archivo G-code: '%s'", s_current_filepath);

    char path[160];
    if (strncmp(s_current_filepath, "/sdcard/", 8) == 0) {
        snprintf(path, sizeof(path), "%s", s_current_filepath);
    } else if (strncmp(s_current_filepath, "/flash/", 7) == 0) {
        snprintf(path, sizeof(path), "/sdcard/%s", s_current_filepath + 7);
    } else {
        snprintf(path, sizeof(path), "/sdcard/%s", s_current_filepath);
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        f = fopen(s_current_filepath, "r");
    }

    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir el archivo '%s' ni '%s'", path, s_current_filepath);
        s_is_running = false;
        s_streamer_task = NULL;
        if (s_complete_cb) s_complete_cb(false);
        vTaskDelete(NULL);
        return;
    }

    // Contar líneas útiles para porcentaje de progreso
    uint32_t total_lines = 0;
    char line_buf[128];
    while (fgets(line_buf, sizeof(line_buf), f)) {
        char *p = line_buf;
        while (isspace((unsigned char)*p)) p++;
        if (*p != '\0' && *p != ';' && *p != '(') {
            total_lines++;
        }
    }
    if (total_lines == 0) total_lines = 1;
    s_total_lines = total_lines;
    s_last_pct = 0.0f;
    s_last_executed = 0;
    s_last_elapsed_sec = 0;
    fseek(f, 0, SEEK_SET);

    uint32_t lines_done = 0;
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_ui_update_us = 0;
    char resp[32];

    while (!s_stop_requested && fgets(line_buf, sizeof(line_buf), f)) {
        // Pausa
        while (s_is_paused && !s_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (s_stop_requested) break;

        // Limpiar fin de línea y verificar si es útil
        char *p = line_buf;
        while (isspace((unsigned char)*p)) p++;
        if (*p == '\0' || *p == ';' || *p == '(') continue;

        size_t len = strlen(p);
        while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == '\n')) {
            p[--len] = '\0';
        }

        // Control de flujo: esperar si el buffer de lookahead está casi lleno (>= 24 bloques)
        while (planner_get_block_count() >= 24 && !s_stop_requested) {
            if (s_is_paused) break;
            vTaskDelay(pdMS_TO_TICKS(20));
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_ui_update_us >= 100000) {
                last_ui_update_us = now_us;
                uint32_t elapsed_sec = (uint32_t)((now_us - start_time_us) / 1000000);
                uint32_t in_buf = planner_get_block_count();
                uint32_t executed = (lines_done > in_buf) ? (lines_done - in_buf) : lines_done;
                float pct = ((float)executed / (float)total_lines) * 100.0f;
                if (pct > 99.0f && lines_done < total_lines) pct = 99.0f;

                s_last_pct = pct;
                s_last_executed = executed;
                s_last_elapsed_sec = elapsed_sec;

                if (s_progress_cb) {
                    s_progress_cb(pct, executed, total_lines, elapsed_sec);
                }
            }
        }
        if (s_stop_requested) break;

        // Enviar comando G-code al planificador
        gcode_parser_execute_line(p, resp, sizeof(resp));

        lines_done++;

        // Actualizar telemetría de UI cada 100 ms o en la última línea
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_ui_update_us >= 100000 || lines_done >= total_lines) {
            last_ui_update_us = now_us;
            uint32_t elapsed_sec = (uint32_t)((now_us - start_time_us) / 1000000);
            uint32_t in_buf = planner_get_block_count();
            uint32_t executed = (lines_done > in_buf) ? (lines_done - in_buf) : lines_done;
            float pct = ((float)executed / (float)total_lines) * 100.0f;
            if (pct > 99.0f && lines_done < total_lines) pct = 99.0f;

            s_last_pct = pct;
            s_last_executed = executed;
            s_last_elapsed_sec = elapsed_sec;

            if (s_progress_cb) {
                s_progress_cb(pct, executed, total_lines, elapsed_sec);
            }
        }
    }

    fclose(f);

    bool success = (!s_stop_requested);

    if (success) {
        // Esperar a que los motores terminen físicamente todos los bloques planificados
        ESP_LOGI(TAG, "Líneas de archivo procesadas (%u). Esperando finalización de movimientos...", (unsigned)lines_done);
        while (planner_get_block_count() > 0 && !s_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(100));
            int64_t now_us = esp_timer_get_time();
            uint32_t elapsed_sec = (uint32_t)((now_us - start_time_us) / 1000000);
            uint32_t in_buf = planner_get_block_count();
            uint32_t executed = (total_lines > in_buf) ? (total_lines - in_buf) : total_lines;
            float pct = ((float)executed / (float)total_lines) * 100.0f;
            if (pct > 99.0f) pct = 99.0f;
            s_last_pct = pct;
            s_last_executed = executed;
            s_last_elapsed_sec = elapsed_sec;
            if (s_progress_cb) {
                s_progress_cb(pct, executed, total_lines, elapsed_sec);
            }
        }

        laser_set_power(0);
        laser_set_enable(false);
        buzzer_play_job_complete();

        int64_t end_us = esp_timer_get_time();
        uint32_t total_sec = (uint32_t)((end_us - start_time_us) / 1000000);
        s_last_pct = 100.0f;
        s_last_executed = total_lines;
        s_last_elapsed_sec = total_sec;
        if (s_progress_cb) {
            s_progress_cb(100.0f, total_lines, total_lines, total_sec);
        }
        ESP_LOGI(TAG, "Trabajo de corte completado exitosamente en %u segundos.", (unsigned)total_sec);
    } else {
        ESP_LOGW(TAG, "Trabajo de corte abortado.");
        planner_abort();
        laser_set_power(0);
        laser_set_enable(false);
    }

    s_is_running = false;
    s_is_paused = false;
    s_stop_requested = false;
    s_streamer_task = NULL;

    if (s_complete_cb) {
        s_complete_cb(success);
    }

    vTaskDelete(NULL);
}

bool gcode_streamer_start(const char *filepath, 
                          gcode_streamer_progress_cb_t progress_cb,
                          gcode_streamer_complete_cb_t complete_cb)
{
    if (!filepath || strlen(filepath) == 0) return false;
    if (s_is_running) {
        ESP_LOGW(TAG, "Ya hay un trabajo de corte en ejecucion.");
        return false;
    }

    strncpy(s_current_filepath, filepath, sizeof(s_current_filepath) - 1);
    s_progress_cb = progress_cb;
    s_complete_cb = complete_cb;
    s_is_running = true;
    s_is_paused = false;
    s_stop_requested = false;
    laser_set_enable(true);

#if defined(ESP_PLATFORM)
    BaseType_t res = xTaskCreatePinnedToCore(
        task_streamer,
        "task_streamer",
        4096,
        NULL,
        4, // Prioridad 4 (Igual a LVGL para evitar inanición de CPU en Core 0)
        &s_streamer_task,
        0
    );
#else
    BaseType_t res = xTaskCreate(
        task_streamer,
        "task_streamer",
        4096,
        NULL,
        4,
        &s_streamer_task
    );
#endif

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Fallo al crear la tarea de streaming FreeRTOS");
        s_is_running = false;
        return false;
    }

    return true;
}

void gcode_streamer_pause(bool pause)
{
    s_is_paused = pause;
    ESP_LOGI(TAG, "Streaming %s", pause ? "PAUSADO" : "REANUDADO");
}

void gcode_streamer_stop(void)
{
    ESP_LOGW(TAG, "Parada inmediata solicitada para el corte G-code.");
    s_stop_requested = true;
    s_is_paused = false;

    // 1. Apagado instantáneo del cabezal láser
    laser_set_power(0);
    laser_set_enable(false);

    // 2. Parada instantánea de emisión de pasos de motores
    stepper_timer_stop();

    // 3. Purgado y vaciado del buffer cinemático
    planner_abort();

    // 4. Retornar a estado IDLE seguro
    stepper_timer_set_state(CNC_STATE_IDLE);
}

bool gcode_streamer_is_running(void)
{
    return s_is_running;
}

bool gcode_streamer_is_paused(void)
{
    return s_is_paused;
}

void gcode_streamer_get_status(gcode_streamer_status_t *out_status)
{
    if (!out_status) return;
    out_status->is_running = s_is_running;
    out_status->is_paused = s_is_paused;
    strncpy(out_status->filepath, s_current_filepath, sizeof(out_status->filepath) - 1);
    out_status->filepath[sizeof(out_status->filepath) - 1] = '\0';
    out_status->progress_pct = s_last_pct;
    out_status->current_line = s_last_executed;
    out_status->total_lines = s_total_lines;
    out_status->elapsed_sec = s_last_elapsed_sec;
}
