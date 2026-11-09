/**
 * @file buzzer_driver.h
 * @brief Driver de avisos acústicos no-bloqueante para zumbador pasivo en GPIO 38.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el pin GPIO 38 y el canal LEDC para generación de tonos acústicos.
 */
esp_err_t buzzer_init(void);

/**
 * @brief Emite un tono breve con frecuencia y duración especificadas.
 * @param freq_hz Frecuencia en Hz (e.g. 2000 - 3000 Hz)
 * @param duration_ms Duración en milisegundos
 */
void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms);

/**
 * @brief Emite la melodía de finalización de corte (3 beeps suaves de 100ms).
 */
void buzzer_play_job_complete(void);

/**
 * @brief Emite un patrón sonoro de advertencia/alarma de seguridad (tono intermitente rápido).
 */
void buzzer_play_alert(void);

/**
 * @brief Detiene cualquier emisión acústica inmediatamente.
 */
void buzzer_stop(void);

#ifdef __cplusplus
}
#endif

