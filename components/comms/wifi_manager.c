/**
 * @file wifi_manager.c
 * @brief Pure Station Wi-Fi Manager for Focal CNC (No SoftAP)
 */

#include <string.h>
#include "wifi_manager.h"
#include "discovery_service.h"
#include "settings_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "WIFI_MGR";

static bool s_is_active = false;
static bool s_sta_connected = false;
static bool s_sta_connecting = false;
static char s_sta_ip[32] = "Sin conexion";
static char s_sta_ssid[33] = {0};
static int8_t s_sta_rssi = 0;
static esp_netif_t *s_sta_netif = NULL;
static SemaphoreHandle_t s_wifi_mutex = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "Wi-Fi Station arrancada. Lista para escanear/conectar.");
                break;
            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
                snprintf(s_sta_ssid, sizeof(s_sta_ssid), "%s", event->ssid);
                s_sta_connecting = true;
                ESP_LOGI(TAG, "Enlazado a AP: '%s'. Obteniendo IP DHCP...", s_sta_ssid);
                break;
            }
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
                s_sta_connected = false;
                s_sta_connecting = false;
                if (disc->reason == 201) {
                    snprintf(s_sta_ip, sizeof(s_sta_ip), "Red fuera de alcance");
                } else if (disc->reason == 15 || disc->reason == 202) {
                    snprintf(s_sta_ip, sizeof(s_sta_ip), "Clave incorrecta");
                } else {
                    snprintf(s_sta_ip, sizeof(s_sta_ip), "Desconectado");
                }
                ESP_LOGW(TAG, "Wi-Fi Station desconectada (razon: %d)", disc->reason);
                break;
            }
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&event->ip_info.ip));
            s_sta_connected = true;
            s_sta_connecting = false;

            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                s_sta_rssi = ap_info.rssi;
            }
            ESP_LOGI(TAG, ">>> Conectado exitosamente! IP: %s (SSID: '%s', RSSI: %d dBm) <<<",
                     s_sta_ip, s_sta_ssid, s_sta_rssi);

            // Iniciar servicio de auto-descubrimiento en la red local
            discovery_service_start();
        }
    }
}

void wifi_manager_init(void)
{
    ESP_LOGI(TAG, "Iniciando Wi-Fi en modo Station exclusivo (sin SoftAP)...");

    if (!s_wifi_mutex) {
        s_wifi_mutex = xSemaphoreCreateMutex();
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();

    // Solo creamos la interfaz Station (conectarse a routers del usuario)
    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_sta_netif, "focal-cnc");

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Modo Station exclusivo
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    s_is_active = true;

    // Conectar automáticamente si hay credenciales guardadas en NVS
    machine_settings_t settings;
    settings_manager_get(&settings);
    if (settings.wifi_sta_enabled && strlen(settings.wifi_ssid) > 0) {
        ESP_LOGI(TAG, "Conectando a red guardada: '%s'...", settings.wifi_ssid);
        wifi_manager_connect_sta(settings.wifi_ssid, settings.wifi_pass);
    }
}

bool wifi_manager_is_connected(void)
{
    return s_sta_connected;
}

const char *wifi_manager_get_ap_ip(void)
{
    return "SoftAP Desactivado";
}

const char *wifi_manager_get_ip(void)
{
    return s_sta_ip;
}

esp_err_t wifi_manager_scan(wifi_scan_item_t *out_list, uint16_t max_items, uint16_t *out_count)
{
    if (!out_list || !out_count || max_items == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_count = 0;

    if (xSemaphoreTake(s_wifi_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Iniciando escaneo activo de redes Wi-Fi locales...");

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 300,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al iniciar escaneo: %s", esp_err_to_name(err));
        xSemaphoreGive(s_wifi_mutex);
        return err;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count > max_items) ap_count = max_items;

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (!ap_records) {
        xSemaphoreGive(s_wifi_mutex);
        return ESP_ERR_NO_MEM;
    }

    err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (err == ESP_OK) {
        *out_count = ap_count;
        for (uint16_t i = 0; i < ap_count; i++) {
            strncpy(out_list[i].ssid, (const char *)ap_records[i].ssid, sizeof(out_list[i].ssid) - 1);
            out_list[i].ssid[sizeof(out_list[i].ssid) - 1] = '\0';
            out_list[i].rssi = ap_records[i].rssi;
            out_list[i].authmode = (uint8_t)ap_records[i].authmode;
            ESP_LOGI(TAG, "  [%02d] SSID: '%s' | RSSI: %d dBm | Auth: %u",
                     i, out_list[i].ssid, out_list[i].rssi, out_list[i].authmode);
        }
    }

    free(ap_records);
    xSemaphoreGive(s_wifi_mutex);
    return err;
}

esp_err_t wifi_manager_connect_sta(const char *ssid, const char *password)
{
    if (!ssid || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_wifi_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Conectando a SSID '%s'...", ssid);

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    }
    // Aceptar cualquier tipo de autenticacion (Open, WPA, WPA2, WPA3, WEP)
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    s_sta_connected = false;
    s_sta_connecting = true;
    snprintf(s_sta_ip, sizeof(s_sta_ip), "Conectando...");
    strncpy(s_sta_ssid, ssid, sizeof(s_sta_ssid) - 1);

    esp_wifi_disconnect();
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err == ESP_OK) {
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error en esp_wifi_connect: %s", esp_err_to_name(err));
            s_sta_connecting = false;
            snprintf(s_sta_ip, sizeof(s_sta_ip), "Error conexion");
        }
    } else {
        s_sta_connecting = false;
    }

    xSemaphoreGive(s_wifi_mutex);
    return err;
}

esp_err_t wifi_manager_disconnect_sta(void)
{
    s_sta_connected = false;
    s_sta_connecting = false;
    snprintf(s_sta_ip, sizeof(s_sta_ip), "Desconectado");
    s_sta_ssid[0] = '\0';
    return esp_wifi_disconnect();
}

bool wifi_manager_is_sta_connected(void)
{
    return s_sta_connected;
}

bool wifi_manager_is_sta_connecting(void)
{
    return s_sta_connecting;
}

const char *wifi_manager_get_sta_ip(void)
{
    return s_sta_ip;
}

const char *wifi_manager_get_sta_ssid(void)
{
    return s_sta_ssid;
}

int8_t wifi_manager_get_sta_rssi(void)
{
    if (s_sta_connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_sta_rssi = ap_info.rssi;
        }
    }
    return s_sta_rssi;
}
