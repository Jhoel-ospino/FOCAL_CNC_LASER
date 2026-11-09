/**
 * @file sd_card.h
 * @brief Montaje y gestión de sistema de archivos FATFS en MicroSD (/sdcard).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_MOUNT_POINT "/sdcard"

typedef struct {
    char name[64];
    size_t size_bytes;
    bool is_directory;
} sd_file_info_t;

/**
 * @brief Inicializa el host SPI y monta el sistema de archivos FATFS en /sdcard.
 */
esp_err_t sd_card_init(void);

/**
 * @brief Retorna si la tarjeta SD está montada y lista para lectura/escritura.
 */
bool sd_card_is_mounted(void);

/**
 * @brief Obtiene una lista de archivos .gcode / .nc en formato JSON para la Web UI y LVGL.
 */
esp_err_t sd_card_get_file_list_json(char *buffer, size_t max_len);
esp_err_t sd_card_list_files_json(char *buffer, size_t max_len);

/**
 * @brief Formatea la tarjeta o partición de almacenamiento.
 */
esp_err_t sd_card_format(void);

#ifdef __cplusplus
}
#endif
