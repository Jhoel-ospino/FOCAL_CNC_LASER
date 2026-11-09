/**
 * @file planner.c
 * @brief Real-time Kinematic Motion Planner with 32-block Lookahead Ring Buffer & GRBL Junction Deviation
 */

#include <math.h>
#include <string.h>
#include "planner.h"
#include "settings_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "PLANNER";

#define JUNCTION_DEVIATION 0.015f // mm (GRBL $11 standard cornering tolerance)

static plan_block_t s_block_buffer[PLANNER_BUFFER_SIZE];
static uint32_t s_head = 0;
static uint32_t s_tail = 0;
static uint32_t s_count = 0;
static SemaphoreHandle_t s_mutex = NULL;

static point_2d_t s_current_pos = {0.0f, 0.0f};
static float s_prev_unit_vec_x = 0.0f;
static float s_prev_unit_vec_y = 0.0f;

void planner_init(void)
{
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
    }
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_current_pos.x = 0.0f;
    s_current_pos.y = 0.0f;
    s_prev_unit_vec_x = 0.0f;
    s_prev_unit_vec_y = 0.0f;

    ESP_LOGI(TAG, "Planner inicializado (Buffer Lookahead: %d bloques, Cama por defecto: %.0fx%.0f mm)", 
             PLANNER_BUFFER_SIZE, BED_MAX_X, BED_MAX_Y);
}

static void planner_recalculate_lookahead(void)
{
    // Trapezoidal lookahead backward/forward pass
    if (s_count <= 1) return;

    // Backward pass
    for (int i = (int)s_count - 1; i > 0; i--) {
        uint32_t idx_curr = (s_tail + i) % PLANNER_BUFFER_SIZE;
        uint32_t idx_prev = (s_tail + i - 1) % PLANNER_BUFFER_SIZE;

        plan_block_t *curr = &s_block_buffer[idx_curr];
        plan_block_t *prev = &s_block_buffer[idx_prev];

        // Velocidad máxima admisible de entrada basada en la distancia de desaceleración hasta la salida
        float max_entry = sqrtf(curr->exit_speed * curr->exit_speed + 2.0f * curr->acceleration * curr->millimeters);
        if (curr->entry_speed > max_entry) curr->entry_speed = max_entry;
        if (curr->entry_speed > curr->nominal_speed) curr->entry_speed = curr->nominal_speed;

        if (prev->exit_speed > curr->entry_speed) {
            prev->exit_speed = curr->entry_speed;
        }
    }

    // Forward pass
    for (uint32_t i = 0; i < s_count; i++) {
        uint32_t idx = (s_tail + i) % PLANNER_BUFFER_SIZE;
        plan_block_t *blk = &s_block_buffer[idx];

        float max_exit = sqrtf(blk->entry_speed * blk->entry_speed + 2.0f * blk->acceleration * blk->millimeters);
        if (blk->exit_speed > max_exit) blk->exit_speed = max_exit;
        if (blk->exit_speed > blk->nominal_speed) blk->exit_speed = blk->nominal_speed;
    }
}

bool planner_buffer_line(float target_x, float target_y, float feedrate, uint16_t laser_power, bool dynamic_mode)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return false;

    if (s_count >= PLANNER_BUFFER_SIZE) {
        xSemaphoreGive(s_mutex);
        return false; // Buffer full
    }

    // Obtener límites y resolución de configuraciones persistentes (NVS)
    float max_x = BED_MAX_X;
    float max_y = BED_MAX_Y;
    float steps_x_per_mm = DEFAULT_STEPS_PER_MM;
    float steps_y_per_mm = DEFAULT_STEPS_PER_MM;
    float default_accel = DEFAULT_ACCELERATION;

    machine_settings_t cfg;
    settings_manager_get(&cfg);
    if (cfg.travel_max_x > 0.0f) max_x = cfg.travel_max_x;
    if (cfg.travel_max_y > 0.0f) max_y = cfg.travel_max_y;
    if (cfg.steps_per_mm_x > 0.0f) steps_x_per_mm = cfg.steps_per_mm_x;
    if (cfg.steps_per_mm_y > 0.0f) steps_y_per_mm = cfg.steps_per_mm_y;
    if (cfg.accel_x > 0.0f) default_accel = cfg.accel_x;

    // Soft limits clamp
    if (target_x < 0.0f) target_x = 0.0f;
    if (target_x > max_x) target_x = max_x;
    if (target_y < 0.0f) target_y = 0.0f;
    if (target_y > max_y) target_y = max_y;

    float dx = target_x - s_current_pos.x;
    float dy = target_y - s_current_pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.001f) {
        // Bloque de desplazamiento nulo
        s_current_pos.x = target_x;
        s_current_pos.y = target_y;
        xSemaphoreGive(s_mutex);
        return true;
    }

    plan_block_t *blk = &s_block_buffer[s_head];
    memset(blk, 0, sizeof(plan_block_t));

    blk->millimeters = dist;
    blk->acceleration = default_accel;
    blk->nominal_speed = (feedrate > 0.0f ? feedrate : DEFAULT_MAX_FEEDRATE) / 60.0f; // mm/s
    if (blk->nominal_speed > (DEFAULT_MAX_FEEDRATE / 60.0f)) {
        blk->nominal_speed = DEFAULT_MAX_FEEDRATE / 60.0f;
    }

    // Calcular pulsos para husillos trapezoidales T8x8
    blk->dir_x = (dx >= 0.0f) ? 1 : 0;
    blk->dir_y = (dy >= 0.0f) ? 1 : 0;
    blk->steps_x = (uint32_t)(fabsf(dx) * steps_x_per_mm + 0.5f);
    blk->steps_y = (uint32_t)(fabsf(dy) * steps_y_per_mm + 0.5f);
    blk->step_event_count = (blk->steps_x > blk->steps_y) ? blk->steps_x : blk->steps_y;

    blk->laser_power = laser_power;
    blk->dynamic_laser = dynamic_mode;

    // Vector unitario del segmento
    float unit_vec_x = dx / dist;
    float unit_vec_y = dy / dist;

    // Cálculo de Junction Deviation de GRBL (Velocidad óptima de paso por esquina)
    if (s_count == 0) {
        // Arranca desde reposo
        blk->entry_speed = 0.0f;
    } else {
        // Coseno del ángulo entre el vector anterior y el actual
        float cos_theta = -(s_prev_unit_vec_x * unit_vec_x + s_prev_unit_vec_y * unit_vec_y);
        if (cos_theta > 0.999999f) {
            // Ángulo agudo de 180° (reversa completa): detener a cero
            blk->entry_speed = 0.0f;
        } else {
            if (cos_theta < -0.999999f) cos_theta = -0.999999f;
            float sin_theta_d2 = sqrtf(0.5f * (1.0f - cos_theta));
            if (sin_theta_d2 < 0.999999f) {
                float r = (JUNCTION_DEVIATION * sin_theta_d2) / (1.0f - sin_theta_d2);
                float max_junction_speed = sqrtf(blk->acceleration * r);
                blk->entry_speed = (max_junction_speed < blk->nominal_speed) ? max_junction_speed : blk->nominal_speed;
            } else {
                blk->entry_speed = blk->nominal_speed;
            }
        }
    }
    blk->exit_speed = 0.0f;

    s_prev_unit_vec_x = unit_vec_x;
    s_prev_unit_vec_y = unit_vec_y;

    s_head = (s_head + 1) % PLANNER_BUFFER_SIZE;
    s_count++;

    // Actualizar posición estimada
    s_current_pos.x = target_x;
    s_current_pos.y = target_y;

    // Recalcular perfiles trapezoidales
    planner_recalculate_lookahead();

    xSemaphoreGive(s_mutex);
    return true;
}

plan_block_t *planner_get_current_block(void)
{
    if (s_count == 0) return NULL;
    return &s_block_buffer[s_tail];
}

void planner_discard_current_block(void)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return;
    if (s_count > 0) {
        s_tail = (s_tail + 1) % PLANNER_BUFFER_SIZE;
        s_count--;
    }
    xSemaphoreGive(s_mutex);
}

uint32_t planner_get_block_count(void)
{
    return s_count;
}

void planner_set_position(float x, float y)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return;
    s_current_pos.x = x;
    s_current_pos.y = y;
    s_prev_unit_vec_x = 0.0f;
    s_prev_unit_vec_y = 0.0f;
    xSemaphoreGive(s_mutex);
}

void planner_get_position(float *x, float *y)
{
    if (x) *x = s_current_pos.x;
    if (y) *y = s_current_pos.y;
}

void planner_abort(void)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return;
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_prev_unit_vec_x = 0.0f;
    s_prev_unit_vec_y = 0.0f;
    xSemaphoreGive(s_mutex);
    ESP_LOGW(TAG, "Buffer del planificador abortado y vaciado.");
}
