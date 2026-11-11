/**
 * @file settings_manager.h
 * @brief Administrador de configuraciones persistentes en memoria Flash (NVS)
 *        para Cortadora Láser Focal CNC.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_MAX_SAVED_WIFI 5

typedef struct {
    char ssid[32];
    char pass[64];
} wifi_saved_ap_t;

typedef struct {
    // 1. Conectividad Wi-Fi
    char wifi_ssid[32];
    char wifi_pass[64];
    bool wifi_sta_enabled;       // true: Conectar a Wi-Fi del usuario, false: Solo SoftAP
    wifi_saved_ap_t saved_wifis[SETTINGS_MAX_SAVED_WIFI];
    uint8_t saved_wifis_count;

    // 2. Gestión de Archivos en SD
    bool sd_auto_overwrite;      // true: Sobrescribir archivos duplicados
    uint8_t sd_sort_mode;        // 0: Por nombre A-Z, 1: Más recientes primero
    char sd_default_folder[32];  // Carpeta de inicio (e.g. "/gcode")

    // 3. Parámetros del Módulo Láser
    uint16_t laser_max_s;        // Valor S máximo de potencia (default: 1000)
    uint16_t laser_pwm_freq;     // Frecuencia PWM en Hz (default: 5000)
    bool laser_air_assist;       // true: Asistencia de aire automática con M8
    uint16_t laser_delay_ms;     // Retardo de estabilización del haz (default: 50 ms)
    uint8_t laser_test_power;    // Potencia del puntero de encuadre en % (default: 1%)

    // 4. Parámetros Cinemáticos y Límites (GRBL)
    float steps_per_mm_x;        // default: 205.13 steps/mm (calibrado T8x8 1/8)
    float steps_per_mm_y;        // default: 200.0 steps/mm (calibrado T8x8 1/8)
    float max_rate_x;            // default: 6000.0 mm/min
    float max_rate_y;            // default: 6000.0 mm/min
    float accel_x;               // default: 500.0 mm/s^2
    float accel_y;               // default: 500.0 mm/s^2
    float travel_max_x;          // default: 160.0 mm
    float travel_max_y;          // default: 200.0 mm
    int16_t bed_rotation;        // default: 0 deg (0, 90, 180, 270)

    // 5. Calibración Óptica
    float focal_z_height;        // Altura Z de enfoque óptimo (default: 30.0 mm)

    // 6. Apariencia y Tema de UI
    uint8_t theme_mode;          // 0: Modo Oscuro, 1: Modo Claro
    uint8_t accent_color_idx;    // 0: Blue, 1: Mint, 2: Amber, 3: Purple, 4: Red

    // 7. Favoritos de Corte Rapido
    char favorite_files[2][64];
    uint8_t num_favorites;
} machine_settings_t;

/**
 * @brief Inicializa el subsistema NVS y carga las configuraciones guardadas.
 *        Si no existen configuraciones previas, carga los valores de fábrica.
 */
esp_err_t settings_manager_init(void);

/**
 * @brief Obtiene una copia de la configuración actual en memoria.
 */
void settings_manager_get(machine_settings_t *out_settings);

/**
 * @brief Guarda la configuración en la memoria flash NVS del ESP32-S3.
 */
esp_err_t settings_manager_save(const machine_settings_t *settings);

/**
 * @brief Restablece todas las configuraciones a sus valores de fábrica predeterminados
 *        y las almacena en la memoria flash NVS.
 */
esp_err_t settings_manager_reset_defaults(void);

/**
 * @brief Busca si una red Wi-Fi ya tiene credenciales guardadas.
 */
bool settings_manager_find_wifi(const char *ssid, char *out_pass, size_t max_len);

/**
 * @brief Agrega o actualiza una red Wi-Fi en la lista de redes guardadas.
 */
void settings_manager_add_or_update_wifi(const char *ssid, const char *pass);

/**
 * @brief Olvida únicamente la red Wi-Fi actualmente conectada.
 */
void settings_manager_forget_current_wifi(void);

#ifdef __cplusplus
}
#endif
