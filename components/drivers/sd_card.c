/**
 * @file sd_card.c
 * @brief Implementación FATFS sobre bus SPI para MicroSD (/sdcard)
 *        con fallback automático a SPI Flash interna (partición "storage").
 */

#include "sd_card.h"
#include "pinout.h"
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "wear_levelling.h"
#include "esp_log.h"

static const char *TAG = "SD_CARD";
static bool s_is_mounted = false;
static sdmmc_card_t *s_card = NULL;
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

extern const uint8_t prueba_mario_gcode_start[] asm("_binary_prueba_mario_gcode_start");
extern const uint8_t prueba_mario_gcode_end[]   asm("_binary_prueba_mario_gcode_end");

static void ensure_test_file_present(void)
{
    struct stat st;
    if (stat(SD_MOUNT_POINT "/prueba_mario.gcode", &st) != 0 || st.st_size == 0) {
        size_t size = prueba_mario_gcode_end - prueba_mario_gcode_start;
        ESP_LOGI(TAG, "Cargando archivo de prueba 'prueba_mario.gcode' (%u bytes) en %s...",
                 (unsigned int)size, SD_MOUNT_POINT);
        FILE *f = fopen(SD_MOUNT_POINT "/prueba_mario.gcode", "wb");
        if (f) {
            fwrite(prueba_mario_gcode_start, 1, size, f);
            fclose(f);
            ESP_LOGI(TAG, "'prueba_mario.gcode' instalado exitosamente en %s.", SD_MOUNT_POINT);
        } else {
            ESP_LOGE(TAG, "Error al escribir %s/prueba_mario.gcode", SD_MOUNT_POINT);
        }
    } else {
        ESP_LOGI(TAG, "'prueba_mario.gcode' ya existe en %s (%ld bytes).", SD_MOUNT_POINT, st.st_size);
    }
}

esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "Montando sistema de archivos FATFS en %s...", SD_MOUNT_POINT);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_SD_CS;
    slot_config.host_id = SPI2_HOST;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MicroSD fisica no detectada (%s). Emulando SD en memoria Flash interna (particion 'storage')...",
                 esp_err_to_name(ret));

        esp_vfs_fat_mount_config_t flash_mount_cfg = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
        };

        ret = esp_vfs_fat_spiflash_mount_rw_wl(SD_MOUNT_POINT, "storage", &flash_mount_cfg, &s_wl_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al montar FATFS en Flash SPI: %s", esp_err_to_name(ret));
            s_is_mounted = false;
            return ret;
        }
    }

    s_is_mounted = true;
    ESP_LOGI(TAG, "Sistema de archivos montado con exito en %s.", SD_MOUNT_POINT);

    ensure_test_file_present();
    return ESP_OK;
}

bool sd_card_is_mounted(void)
{
    return s_is_mounted;
}

esp_err_t sd_card_format(void)
{
    ESP_LOGW(TAG, "Formateando sistema de archivos en %s...", SD_MOUNT_POINT);
    if (s_wl_handle != WL_INVALID_HANDLE) {
        esp_vfs_fat_spiflash_unmount_rw_wl(SD_MOUNT_POINT, s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
        s_is_mounted = false;

        esp_vfs_fat_mount_config_t flash_mount_cfg = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
        };
        esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl(SD_MOUNT_POINT, "storage", &flash_mount_cfg, &s_wl_handle);
        if (ret == ESP_OK) {
            s_is_mounted = true;
            ensure_test_file_present();
        }
        return ret;
    }
    return ESP_OK;
}

esp_err_t sd_card_get_file_list_json(char *buffer, size_t max_len)
{
    if (!buffer || max_len < 16) return ESP_ERR_INVALID_ARG;

    if (!s_is_mounted) {
        snprintf(buffer, max_len, "{\"mounted\":false,\"files\":[]}");
        return ESP_OK;
    }

    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) {
        snprintf(buffer, max_len, "{\"mounted\":false,\"files\":[]}");
        return ESP_FAIL;
    }

    size_t written = snprintf(buffer, max_len, "{\"mounted\":true,\"files\":[");
    struct dirent *entry;
    bool first = true;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN) {
            if (entry->d_name[0] == '.') continue;

            char filepath[300];
            snprintf(filepath, sizeof(filepath), "%s/%s", SD_MOUNT_POINT, entry->d_name);
            struct stat st;
            size_t size = 0;
            if (stat(filepath, &st) == 0) {
                size = st.st_size;
            }

            char item[384];
            snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"size\":%u}",
                     first ? "" : ",", entry->d_name, (unsigned int)size);
            first = false;

            if (written + strlen(item) + 4 < max_len) {
                written += snprintf(buffer + written, max_len - written, "%s", item);
            } else {
                break;
            }
        }
    }
    closedir(dir);

    snprintf(buffer + written, max_len - written, "]}");
    return ESP_OK;
}

esp_err_t sd_card_list_files_json(char *buffer, size_t max_len)
{
    return sd_card_get_file_list_json(buffer, max_len);
}
