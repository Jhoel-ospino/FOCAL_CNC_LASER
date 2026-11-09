/**
 * @file tmc2209.h
 * @brief Driver Trinamic TMC2209 en modo Standalone (STEP/DIR + Enable común).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TMC2209_AXIS_X          0
#define TMC2209_AXIS_Y          1

/**
 * @brief Inicializa los drivers TMC2209 en modo Standalone (GPIO 3 Enable).
 */
esp_err_t tmc2209_init(void);

/**
 * @brief Simulación de ajuste de corriente en modo standalone (informativo).
 */
esp_err_t tmc2209_set_current(uint8_t axis, uint16_t current_ma);

/**
 * @brief Simulación de ajuste de micropasos en modo standalone (informativo).
 */
esp_err_t tmc2209_set_microsteps(uint8_t axis, uint16_t microsteps);

/**
 * @brief Habilita o deshabilita la etapa de potencia de todos los motores (GPIO 3).
 * @param enable true para habilitar (LOW), false para deshabilitar (HIGH).
 */
esp_err_t tmc2209_enable_drivers(bool enable);

/**
 * @brief Consulta si los drivers de motor están habilitados.
 */
bool tmc2209_is_enabled(void);

#ifdef __cplusplus
}
#endif
