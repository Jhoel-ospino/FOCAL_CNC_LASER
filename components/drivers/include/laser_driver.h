/**
 * @file laser_driver.h
 * @brief Driver de modulación PWM y enclavamiento de seguridad para Tree Laser 20W óptico.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LASER_MAX_S_VAL     1000 // Escala estándar GRBL ($30=1000)

/**
 * @brief Inicializa el canal LEDC de alta frecuencia para control del láser Tree Laser 20W.
 */
esp_err_t laser_init(void);

/**
 * @brief Modula la potencia del láser en tiempo real (S0 - S1000).
 */
void laser_set_power(uint16_t s_value);

/**
 * @brief Enciende o apaga el enclavamiento de seguridad del cabezal láser.
 */
void laser_set_enable(bool enable);

/**
 * @brief Retorna el valor actual de potencia S (0 - 1000).
 */
uint16_t laser_get_power(void);

/**
 * @brief Retorna el porcentaje instantáneo de potencia (0 - 100%).
 */
uint8_t laser_get_percent(void);

#ifdef __cplusplus
}
#endif
