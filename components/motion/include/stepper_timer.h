/**
 * @file stepper_timer.h
 * @brief High-frequency Hardware Step Pulse Generator & Laser Sync
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CNC_STATE_IDLE = 0,
    CNC_STATE_RUN,
    CNC_STATE_HOLD,
    CNC_STATE_ALARM
} cnc_state_t;

/**
 * @brief Inicializa los pines GPIO de STEP/DIR y el temporizador hardware esp_timer.
 */
void stepper_timer_init(void);

/**
 * @brief Inicia o reanuda la generación de pulsos de pasos.
 */
void stepper_timer_start(void);

/**
 * @brief Pausa el temporizador de pasos de forma segura.
 */
void stepper_timer_stop(void);

/**
 * @brief Habilita o deshabilita la etapa de potencia de los motores (GPIO 3).
 * @param enable true para habilitar (LOW), false para liberar/apagar (HIGH).
 */
void stepper_timer_enable_motors(bool enable);

/**
 * @brief Devuelve si los motores se encuentran energizados.
 */
bool stepper_timer_is_motors_enabled(void);

/**
 * @brief Devuelve el estado actual de la máquina CNC.
 */
cnc_state_t stepper_timer_get_state(void);

/**
 * @brief Establece el estado de la máquina CNC (e.g. ALARM, HOLD, RUN).
 */
void stepper_timer_set_state(cnc_state_t state);

#ifdef __cplusplus
}
#endif
