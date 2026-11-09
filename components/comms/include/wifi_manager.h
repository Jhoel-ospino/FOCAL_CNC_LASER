/**
 * @file wifi_manager.h
 * @brief Dual-mode SoftAP + Station Wi-Fi Manager with Real Scanning for Focal CNC
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_MAX_SCAN_AP 16

typedef struct {
    char ssid[33];
    int8_t rssi;
    uint8_t authmode;
} wifi_scan_item_t;

/**
 * @brief Inicializa Wi-Fi en modo dual AP+STA.
 *        SoftAP ("Focal-Laser-CNC", pass: "focal1234", IP: 192.168.4.1)
 *        Station activo para escaneo y conexion a redes Wi-Fi locales.
 */
void wifi_manager_init(void);

/**
 * @brief Devuelve true si la interfaz Wi-Fi general esta activa.
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Obtiene la IP del SoftAP (por defecto 192.168.4.1).
 */
const char *wifi_manager_get_ap_ip(void);

/**
 * @brief Obtiene la IP actual principal (Station si esta conectada, sino SoftAP).
 */
const char *wifi_manager_get_ip(void);

/**
 * @brief Escanea de forma real redes Wi-Fi cercanas usando el radio del ESP32-S3.
 * @param out_list Array donde se almacenaran las redes encontradas.
 * @param max_items Capacidad maxima del array out_list.
 * @param out_count Numero de redes efectivamente encontradas.
 */
esp_err_t wifi_manager_scan(wifi_scan_item_t *out_list, uint16_t max_items, uint16_t *out_count);

/**
 * @brief Conecta como Station a una red Wi-Fi con el SSID y contraseña especificados.
 */
esp_err_t wifi_manager_connect_sta(const char *ssid, const char *password);

/**
 * @brief Desconecta de la red Station.
 */
esp_err_t wifi_manager_disconnect_sta(void);

/**
 * @brief Devuelve true si Station esta conectada con IP asignada.
 */
bool wifi_manager_is_sta_connected(void);

/**
 * @brief Devuelve true si Station esta en proceso de conexion o negociacion DHCP.
 */
bool wifi_manager_is_sta_connecting(void);

/**
 * @brief Obtiene la IP asignada en modo Station (ej. "192.168.1.100" o "Desconectado").
 */
const char *wifi_manager_get_sta_ip(void);

/**
 * @brief Obtiene el SSID al que esta actualmente conectada la Station.
 */
const char *wifi_manager_get_sta_ssid(void);

/**
 * @brief Obtiene el nivel de señal RSSI de la red conectada.
 */
int8_t wifi_manager_get_sta_rssi(void);

#ifdef __cplusplus
}
#endif

