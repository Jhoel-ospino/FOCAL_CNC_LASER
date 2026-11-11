/**
 * @file discovery_service.c
 * @brief UDP Broadcast Discovery Beacon and Responder for Focal CNC
 */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "discovery_service.h"
#include "wifi_manager.h"
#include "stepper_timer.h"
#include "gcode_streamer.h"

static const char *TAG = "DISCOVERY";
static TaskHandle_t s_discovery_task = NULL;
static volatile bool s_running = false;

static void task_udp_discovery(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando servicio de descubrimiento UDP en puerto %d...", FOCAL_DISCOVERY_PORT);

    int sock = -1;
    char rx_buffer[256];
    char tx_buffer[384];

    while (s_running) {
        if (!wifi_manager_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "No se pudo crear socket UDP: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            int broadcast = 1;
            setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

            int reuse = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

            struct timeval tv = { .tv_sec = 2, .tv_usec = 500000 }; // 2.5s timeout
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            struct sockaddr_in bind_addr;
            memset(&bind_addr, 0, sizeof(bind_addr));
            bind_addr.sin_family = AF_INET;
            bind_addr.sin_port = htons(FOCAL_DISCOVERY_PORT);
            bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

            int err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Fallo bind en socket UDP: errno %d", errno);
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            ESP_LOGI(TAG, "Socket UDP enlazado exitosamente al puerto %d", FOCAL_DISCOVERY_PORT);
        }

        // 1. Recibir paquetes entrantes de búsqueda de la aplicación de escritorio
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        const char *ip_str = wifi_manager_get_ip();
        const char *state_str = "Idle";
        if (gcode_streamer_is_running()) {
            state_str = gcode_streamer_is_paused() ? "Hold" : "Run";
        } else {
            switch (stepper_timer_get_state()) {
                case CNC_STATE_RUN:   state_str = "Run"; break;
                case CNC_STATE_HOLD:  state_str = "Hold"; break;
                case CNC_STATE_ALARM: state_str = "Alarm"; break;
                default:              state_str = "Idle"; break;
            }
        }

        snprintf(tx_buffer, sizeof(tx_buffer),
                 "{\"focal_device\":\"Focal CNC Laser\",\"ip\":\"%s\",\"port\":80,"
                 "\"version\":\"1.0.0\",\"storage\":\"flash\",\"bed_w\":102.0,\"bed_h\":82.0,\"status\":\"%s\"}",
                 ip_str, state_str);

        if (len > 0) {
            rx_buffer[len] = '\0';
            // Responder directamente al cliente que envió el probe
            sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            ESP_LOGD(TAG, "Respondido probe UDP a cliente %s: %s", inet_ntoa(source_addr.sin_addr), tx_buffer);
        }

        // 2. Emitir paquete Broadcast periódico a la subred para auto-descubrimiento pasivo
        struct sockaddr_in bcast_addr;
        memset(&bcast_addr, 0, sizeof(bcast_addr));
        bcast_addr.sin_family = AF_INET;
        bcast_addr.sin_port = htons(FOCAL_DISCOVERY_PORT);
        bcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); // 255.255.255.255

        sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
    }

    if (sock >= 0) {
        close(sock);
    }
    s_discovery_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t discovery_service_start(void)
{
    if (s_running || s_discovery_task != NULL) {
        return ESP_OK;
    }

    s_running = true;
    BaseType_t res = xTaskCreatePinnedToCore(
        task_udp_discovery,
        "task_discovery",
        3584,
        NULL,
        2,
        &s_discovery_task,
        0
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Error al crear tarea de descubrimiento UDP");
        s_running = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void discovery_service_stop(void)
{
    s_running = false;
}

