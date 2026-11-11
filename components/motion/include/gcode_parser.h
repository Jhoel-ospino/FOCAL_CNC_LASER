/**
 * @file gcode_parser.h
 * @brief GRBL 1.1 Compatible G-code Parser and Command Processor
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    COORD_MODE_ABSOLUTE = 0,
    COORD_MODE_RELATIVE
} coord_mode_t;

/**
 * @brief Inicializa el parser de G-code con parámetros por defecto.
 */
void gcode_parser_init(void);

/**
 * @brief Procesa una línea de comando G-code o comando del sistema GRBL.
 * @param line Cadena terminada en nulo (e.g. "G1 X10.5 Y20.0 F1200 S500")
 * @param response_buf Buffer para escribir la respuesta GRBL ("ok", "error:XX", o status "<...>")
 * @param max_resp_len Tamaño máximo del buffer de respuesta
 * @return true si el comando fue procesado satisfactoriamente
 */
bool gcode_parser_execute_line(const char *line, char *response_buf, size_t max_resp_len);

/**
 * @brief Genera el reporte de estado GRBL en tiempo real para el comando '?'.
 */
void gcode_parser_get_status_report(char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif
