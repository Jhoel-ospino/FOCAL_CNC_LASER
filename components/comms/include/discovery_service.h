/**
 * @file discovery_service.h
 * @brief UDP Broadcast Discovery Beacon and Responder for Focal CNC
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOCAL_DISCOVERY_PORT 45454

/**
 * @brief Inicia el servicio de descubrimiento UDP en segundo plano (Core 0).
 */
esp_err_t discovery_service_start(void);

/**
 * @brief Detiene el servicio de descubrimiento UDP.
 */
void discovery_service_stop(void);

#ifdef __cplusplus
}
#endif

