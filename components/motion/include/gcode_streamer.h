/**
 * @file gcode_streamer.h
 * @brief FreeRTOS Background G-code File Streaming & Job Execution Engine
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*gcode_streamer_progress_cb_t)(float progress_pct, uint32_t current_line,
                                             uint32_t total_lines, uint32_t elapsed_sec);
typedef void (*gcode_streamer_complete_cb_t)(bool success);

typedef struct {
    bool is_running;
    bool is_paused;
    char filepath[128];
    float progress_pct;
    uint32_t current_line;
    uint32_t total_lines;
    uint32_t elapsed_sec;
} gcode_streamer_status_t;

/**
 * @brief Obtiene el estado actual del streaming y corte en segundo plano.
 */
void gcode_streamer_get_status(gcode_streamer_status_t *out_status);

/**
 * @brief Inicia el streaming de un archivo G-code en segundo plano.
 * @param filepath Nombre o ruta del archivo G-code
 * @param progress_cb Callback de telemetría de corte en tiempo real
 * @param complete_cb Callback al completar o abortar
 * @return true si el streaming inició correctamente
 */
bool gcode_streamer_start(const char *filepath, 
                          gcode_streamer_progress_cb_t progress_cb,
                          gcode_streamer_complete_cb_t complete_cb);

/**
 * @brief Pausa o reanuda la ejecución del archivo.
 */
void gcode_streamer_pause(bool pause);

/**
 * @brief Detiene y cancela la ejecución del archivo de inmediato.
 */
void gcode_streamer_stop(void);

/**
 * @brief Consulta si hay un corte ejecutándose activamente.
 */
bool gcode_streamer_is_running(void);

/**
 * @brief Consulta si el corte está en pausa.
 */
bool gcode_streamer_is_paused(void);

/**
 * @brief Cuenta las líneas útiles de un archivo G-code.
 */
uint32_t gcode_streamer_count_lines(const char *filepath);

#ifdef __cplusplus
}
#endif

