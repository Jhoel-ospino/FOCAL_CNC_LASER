/**
 * @file ili9488_lvgl.h
 * @brief Adaptador oficial esp_lcd para LVGL v8 con panel ILI9488 (RGB666 480x320).
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el panel ILI9488 con esp_lcd y registra el driver de pantalla en LVGL.
 */
esp_err_t ili9488_lvgl_init(void);

/**
 * @brief Toma el semáforo/mutex de LVGL antes de modificar objetos gráficos.
 */
bool lvgl_port_lock(uint32_t timeout_ms);

/**
 * @brief Libera el semáforo/mutex de LVGL tras finalizar operaciones gráficas.
 */
void lvgl_port_unlock(void);

/**
 * @brief Espera a que cualquier transacción DMA pendiente sobre el panel ILI9488 termine.
 */
void ili9488_wait_idle(void);

#ifdef __cplusplus
}
#endif
