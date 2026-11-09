/**
 * @file web_server.h
 * @brief Embedded REST HTTP Server and Focal Web UI
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicia el servidor web HTTP en el puerto 80.
 */
void web_server_start(void);

/**
 * @brief Detiene el servidor web HTTP.
 */
void web_server_stop(void);

#ifdef __cplusplus
}
#endif
