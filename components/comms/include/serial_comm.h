/**
 * @file serial_comm.h
 * @brief USB Serial GRBL 1.1 Protocol and Binary File Transfer Handler
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa la tarea de comunicación serial en UART0 / USB Serial.
 */
void serial_comm_init(void);

/**
 * @brief Tarea FreeRTOS para procesamiento de comandos seriales y streaming.
 */
void serial_comm_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
