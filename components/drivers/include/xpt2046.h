/**
 * @file xpt2046.h
 * @brief Driver para panel táctil resistivo XPT2046 integrado con LVGL v8.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el dispositivo XPT2046 en el bus SPI compartido (SPI2_HOST)
 *        y configura el pin de interrupción T_IRQ.
 */
esp_err_t xpt2046_init(void);

/**
 * @brief Lee las coordenadas táctiles calibradas (0..479, 0..319) y presión Z.
 * @return true si la pantalla está siendo presionada, false si está liberada.
 */
bool xpt2046_get_touch(int *out_screen_x, int *out_screen_y, uint16_t *out_z);

/**
 * @brief Registra el panel táctil XPT2046 como dispositivo de entrada (indev) en LVGL.
 */
void xpt2046_register_lvgl_indev(void);

/**
 * @brief Configura la orientación dinámica del panel táctil.
 */
void xpt2046_set_config(bool swap_xy, bool invert_x, bool invert_y);

/**
 * @brief Obtiene la configuración actual de orientación.
 */
void xpt2046_get_config(bool *out_swap_xy, bool *out_invert_x, bool *out_invert_y);

/**
 * @brief Obtiene la última lectura cruda de ADC y coordenadas mapeadas para depuración.
 */
bool xpt2046_get_last_debug(uint16_t *out_raw_x, uint16_t *out_raw_y, uint16_t *out_z, int *out_sx, int *out_sy);

#ifdef __cplusplus
}
#endif
