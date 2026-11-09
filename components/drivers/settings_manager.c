/**
 * @file settings_manager.c
 * @brief Implementación del almacenamiento persistente en NVS Flash
 *        para la Cortadora Láser Focal CNC.
 */

#include "settings_manager.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "SETTINGS_MGR";
#define NVS_NAMESPACE "focal_laser"
#define NVS_BLOB_KEY  "mach_cfg"

static machine_settings_t s_settings;

static const machine_settings_t s_defaults = {
    // Wi-Fi (Sin redes preestablecidas de fabrica)
    .wifi_ssid = "",
    .wifi_pass = "",
    .wifi_sta_enabled = false,
    .saved_wifis = { { "", "" } },
    .saved_wifis_count = 0,

    // SD File Management
    .sd_auto_overwrite = true,
    .sd_sort_mode = 0, // Por nombre
    .sd_default_folder = "/sdcard",

    // Laser
    .laser_max_s = 1000,
    .laser_pwm_freq = 5000, // 5 kHz
    .laser_air_assist = true,
    .laser_delay_ms = 50,
    .laser_test_power = 1, // 1% para encuadre visible sin quemar

    // Kinematics / GRBL Defaults
    .steps_per_mm_x = 400.0f,
    .steps_per_mm_y = 400.0f,
    .max_rate_x = 6000.0f,
    .max_rate_y = 6000.0f,
    .accel_x = 500.0f,
    .accel_y = 500.0f,
    .travel_max_x = 200.0f,
    .travel_max_y = 160.0f,
    .bed_rotation = 0,

    // Optical Focus
    .focal_z_height = 30.0f,

    // UI Appearance
    .theme_mode = 0,       // 0: Dark Mode
    .accent_color_idx = 0, // 0: Pastel Blue

    // Favoritos (Sin favoritos de fabrica)
    .favorite_files = { "", "" },
    .num_favorites = 0
};

esp_err_t settings_manager_init(void)
{
    ESP_LOGI(TAG, "Inicializando Administrador de Configuraciones NVS...");

    // Inicializar NVS si aún no está inicializado
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando nvs_flash: %s", esp_err_to_name(err));
        s_settings = s_defaults;
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo abrir namespace NVS (%s). Cargando valores por defecto.", esp_err_to_name(err));
        s_settings = s_defaults;
        return ESP_OK;
    }

    size_t required_size = sizeof(machine_settings_t);
    err = nvs_get_blob(handle, NVS_BLOB_KEY, &s_settings, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND || required_size != sizeof(machine_settings_t)) {
        ESP_LOGI(TAG, "No se encontraron configuraciones previas o cambió el tamaño. Guardando valores de fábrica...");
        s_settings = s_defaults;
        nvs_set_blob(handle, NVS_BLOB_KEY, &s_settings, sizeof(machine_settings_t));
        nvs_commit(handle);
    } else if (err == ESP_OK) {
        ESP_LOGI(TAG, "Configuraciones cargadas exitosamente de NVS Flash.");
        ESP_LOGI(TAG, "  Wi-Fi SSID: '%s', STA: %d, Láser Freq: %u Hz, Steps/mm X: %.1f",
                 s_settings.wifi_ssid, s_settings.wifi_sta_enabled,
                 s_settings.laser_pwm_freq, s_settings.steps_per_mm_x);
    } else {
        ESP_LOGE(TAG, "Error leyendo NVS blob: %s. Usando defaults.", esp_err_to_name(err));
        s_settings = s_defaults;
    }

    nvs_close(handle);
    return ESP_OK;
}

void settings_manager_get(machine_settings_t *out_settings)
{
    if (out_settings) {
        memcpy(out_settings, &s_settings, sizeof(machine_settings_t));
    }
}

esp_err_t settings_manager_save(const machine_settings_t *settings)
{
    if (!settings) return ESP_ERR_INVALID_ARG;

    memcpy(&s_settings, settings, sizeof(machine_settings_t));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS para guardar: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, NVS_BLOB_KEY, &s_settings, sizeof(machine_settings_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
        ESP_LOGI(TAG, "Configuraciones guardadas y confirmadas en Flash NVS con éxito.");
    } else {
        ESP_LOGE(TAG, "Error guardando blob en NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    return err;
}

esp_err_t settings_manager_reset_defaults(void)
{
    ESP_LOGW(TAG, "Restableciendo configuraciones a valores de fábrica predeterminados...");
    s_settings = s_defaults;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        nvs_set_blob(handle, NVS_BLOB_KEY, &s_settings, sizeof(machine_settings_t));
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Valores de fábrica restablecidos y persistidos en NVS.");
    return err;
}

bool settings_manager_find_wifi(const char *ssid, char *out_pass, size_t max_len)
{
    if (!ssid || !ssid[0]) return false;
    for (int i = 0; i < s_settings.saved_wifis_count; i++) {
        if (strcmp(s_settings.saved_wifis[i].ssid, ssid) == 0) {
            if (out_pass && max_len > 0) {
                strncpy(out_pass, s_settings.saved_wifis[i].pass, max_len - 1);
                out_pass[max_len - 1] = '\0';
            }
            return true;
        }
    }
    return false;
}

void settings_manager_add_or_update_wifi(const char *ssid, const char *pass)
{
    if (!ssid || !ssid[0]) return;
    for (int i = 0; i < s_settings.saved_wifis_count; i++) {
        if (strcmp(s_settings.saved_wifis[i].ssid, ssid) == 0) {
            if (pass) {
                strncpy(s_settings.saved_wifis[i].pass, pass, sizeof(s_settings.saved_wifis[i].pass) - 1);
                s_settings.saved_wifis[i].pass[sizeof(s_settings.saved_wifis[i].pass) - 1] = '\0';
            }
            settings_manager_save(&s_settings);
            return;
        }
    }
    if (s_settings.saved_wifis_count < SETTINGS_MAX_SAVED_WIFI) {
        strncpy(s_settings.saved_wifis[s_settings.saved_wifis_count].ssid, ssid, sizeof(s_settings.saved_wifis[0].ssid) - 1);
        s_settings.saved_wifis[s_settings.saved_wifis_count].ssid[sizeof(s_settings.saved_wifis[0].ssid) - 1] = '\0';
        strncpy(s_settings.saved_wifis[s_settings.saved_wifis_count].pass, pass ? pass : "", sizeof(s_settings.saved_wifis[0].pass) - 1);
        s_settings.saved_wifis[s_settings.saved_wifis_count].pass[sizeof(s_settings.saved_wifis[0].pass) - 1] = '\0';
        s_settings.saved_wifis_count++;
    } else {
        // Shift left and append
        for (int i = 0; i < SETTINGS_MAX_SAVED_WIFI - 1; i++) {
            s_settings.saved_wifis[i] = s_settings.saved_wifis[i+1];
        }
        strncpy(s_settings.saved_wifis[SETTINGS_MAX_SAVED_WIFI - 1].ssid, ssid, sizeof(s_settings.saved_wifis[0].ssid) - 1);
        s_settings.saved_wifis[SETTINGS_MAX_SAVED_WIFI - 1].ssid[sizeof(s_settings.saved_wifis[0].ssid) - 1] = '\0';
        strncpy(s_settings.saved_wifis[SETTINGS_MAX_SAVED_WIFI - 1].pass, pass ? pass : "", sizeof(s_settings.saved_wifis[0].pass) - 1);
        s_settings.saved_wifis[SETTINGS_MAX_SAVED_WIFI - 1].pass[sizeof(s_settings.saved_wifis[0].pass) - 1] = '\0';
    }
    settings_manager_save(&s_settings);
}

void settings_manager_forget_current_wifi(void)
{
    if (strlen(s_settings.wifi_ssid) > 0) {
        for (int i = 0; i < s_settings.saved_wifis_count; i++) {
            if (strcmp(s_settings.saved_wifis[i].ssid, s_settings.wifi_ssid) == 0) {
                for (int j = i; j < s_settings.saved_wifis_count - 1; j++) {
                    s_settings.saved_wifis[j] = s_settings.saved_wifis[j+1];
                }
                s_settings.saved_wifis_count--;
                break;
            }
        }
    }
    s_settings.wifi_ssid[0] = '\0';
    s_settings.wifi_pass[0] = '\0';
    s_settings.wifi_sta_enabled = false;
    settings_manager_save(&s_settings);
}
