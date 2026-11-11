/**
 * @file planner.h
 * @brief Real-time Kinematic Motion Planner with 32-block Lookahead Ring Buffer
 *
 * Designed for ESP32-S3 Core 1 FreeRTOS real-time motion control.
 * Calculates trapezoidal acceleration profiles and steps for 4-start leadscrews.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLANNER_BUFFER_SIZE  32
#define DEFAULT_STEPS_PER_MM_X 205.13f // Husillo T8x8 (8mm/rev), 200 pasos, 8x microstepping, calibrado a 97.5mm
#define DEFAULT_STEPS_PER_MM_Y 200.0f  // Husillo T8x8 (8mm/rev), 200 pasos, 8x microstepping, calibrado a 100mm
#define DEFAULT_STEPS_PER_MM   200.0f  // Compatibilidad legacy
#define DEFAULT_ACCELERATION 40.0f   // mm/s^2 (rampa suave S-curve anti-ruido para motor X y husillos T8)
#define DEFAULT_MAX_FEEDRATE 800.0f  // mm/min (husillos T8x8 óptimo para evitar mid-band resonance a 12V)
#define BED_MAX_X            102.0f  // 102 mm
#define BED_MAX_Y            82.0f   // 82 mm

typedef struct {
    float x;
    float y;
} point_2d_t;

typedef struct {
    // Kinematic parameters
    uint32_t steps_x;
    uint32_t steps_y;
    uint32_t step_event_count; // max(steps_x, steps_y)
    
    uint8_t dir_x;
    uint8_t dir_y;

    float millimeters;
    float nominal_speed; // mm/s
    float entry_speed;   // mm/s
    float exit_speed;    // mm/s
    float acceleration;  // mm/s^2

    // Laser control
    uint16_t laser_power; // 0..1000
    bool dynamic_laser;   // M4 mode
} plan_block_t;

/**
 * @brief Inicializa el buffer circular y estado cinemático del planificador.
 */
void planner_init(void);

/**
 * @brief Añade una línea cinemática al buffer de lookahead.
 * @param target_x Coordenada destino X en mm
 * @param target_y Coordenada destino Y en mm
 * @param feedrate Velocidad de avance programada en mm/min
 * @param laser_power Potencia del láser (0..1000)
 * @param dynamic_mode true para M4 (láser dinámico), false para M3 (constante)
 * @return true si se encoló con éxito, false si el buffer está lleno
 */
bool planner_buffer_line(float target_x, float target_y, float feedrate, uint16_t laser_power, bool dynamic_mode);

/**
 * @brief Obtiene el bloque actual a ejecutar por el temporizador de pasos.
 */
plan_block_t *planner_get_current_block(void);

/**
 * @brief Descarta el bloque actual cuando se completa su ejecución.
 */
void planner_discard_current_block(void);

/**
 * @brief Devuelve el número de bloques disponibles en el buffer de lookahead.
 */
uint32_t planner_get_block_count(void);

/**
 * @brief Fija la posición actual de la máquina (G92 o Homing).
 */
void planner_set_position(float x, float y);

/**
 * @brief Obtiene la posición actual estimada de la herramienta en mm.
 */
void planner_get_position(float *x, float *y);

/**
 * @brief Vacía el buffer de movimientos inmediatamente (Stop / E-Stop).
 */
void planner_abort(void);

#ifdef __cplusplus
}
#endif
