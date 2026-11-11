/**
 * @file main.c
 * @brief Focal CNC Laser Cutter 400x400 mm Industrial Firmware
 *        ESP32-S3 Dual-Core FreeRTOS Architecture
 *
 * Core 0: Connectivity (Wi-Fi SoftAP, HTTP REST Web UI), Serial Streamer, LVGL UI Task
 * Core 1: Real-time Kinematic Motion Planner, 32-block Lookahead, 20W Dynamic Laser PWM
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

// Drivers
#include "pinout.h"
#include "ili9488_lvgl.h"
#include "laser_driver.h"
#include "buzzer_driver.h"
#include "tmc2209.h"
#include "sd_card.h"
#include "settings_manager.h"
#include "nvs_flash.h"

// UI (focal-design-system)
#include "focal_theme.h"
#include "ui_screens.h"

// Motion (GRBL 1.1)
#include "planner.h"
#include "stepper_timer.h"
#include "gcode_parser.h"

// Comms
#include "wifi_manager.h"
#include "web_server.h"
#include "serial_comm.h"

static const char *TAG = "FOCAL_MAIN";

// =============================================================================
// CORE 0 TASKS: UI, Connectivity & File Transfer
// =============================================================================

/**
 * @brief LVGL Graphics Render Task (Core 0, 30-50 FPS)
 */
static void task_lvgl(void *pvParameters)
{
    ESP_LOGI(TAG, "LVGL Render Task started on Core %d", xPortGetCoreID());

    while (1) {
        uint32_t delay_ms = 10;
        if (lvgl_port_lock(10)) {
            delay_ms = lv_timer_handler();
            lvgl_port_unlock();
        }
        if (delay_ms < 1) delay_ms = 1;
        if (delay_ms > 10) delay_ms = 10;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief Comms & Web Server Supervisor Task (Core 0)
 */
static void task_comms(void *pvParameters)
{
    ESP_LOGI(TAG, "Comms Supervisor Task started on Core %d", xPortGetCoreID());

    // Initialize Wi-Fi SoftAP & mDNS
    wifi_manager_init();

    // Start HTTP REST & Focal Web UI
    web_server_start();

    // Periodic telemetry sync loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        // Status reporting / background health checks
    }
}

// =============================================================================
// CORE 1 TASK: Real-time CNC Motion & Kinematics
// =============================================================================

/**
 * @brief Motion Planner Supervisor Task (Core 1)
 */
static void task_motion_planner(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion Planner Real-Time Task started on Core %d", xPortGetCoreID());

    planner_init();
    stepper_timer_init();
    gcode_parser_init();

    while (1) {
        // Real-time motion supervisor monitoring
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================================
// MAIN ENTRY POINT
// =============================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "   FOCAL CNC 20W LASER CUTTER FIRMWARE v1.0.0    ");
    ESP_LOGI(TAG, "   Target: ESP32-S3 N16R8 (Xtensa Dual-Core)      ");
    ESP_LOGI(TAG, "   Bed Size: 102x82 mm | T8x8 (200 steps/mm)      ");
    ESP_LOGI(TAG, "   Design System: focal-design-system (Apple-like)");
    ESP_LOGI(TAG, "==================================================");

    // 0. Initialize NVS Flash and Settings Manager (Permanent Settings Storage)
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    settings_manager_init();

    // 1. Initialize Laser Driver (LEDC PWM 5 kHz, 10-bit) & Buzzer
    laser_init();
    buzzer_init();

    // 2. Initialize Trinamic TMC2209 Stepper Drivers (UART StealthChop2)
    tmc2209_init();

    // 3. Initialize Local 3.5" TFT Display (ILI9488 RGB666 DMA) and LVGL
    ESP_ERROR_CHECK(ili9488_lvgl_init());

    // 4. Initialize MicroSD Card FATFS (/sdcard) on shared SPI2 bus or Flash emulation
    sd_card_init();

    // 5. Build Focal Design System UI Screens (Dashboard, Jogging, SD, Cut Monitor)
    if (lvgl_port_lock(100)) {
        ui_init();
        lvgl_port_unlock();
    }

    // 6. Initialize USB Serial Communication (UART0 115200 baud)
    serial_comm_init();

    // =========================================================================
    // FreeRTOS Dual-Core Task Allocation
    // =========================================================================

    // Core 0: LVGL Rendering Task (Priority 4)
    xTaskCreatePinnedToCore(
        task_lvgl,
        "task_lvgl",
        8192,
        NULL,
        4,
        NULL,
        0
    );

    // Core 0: Serial Protocol & Streaming Task (Priority 3)
    xTaskCreatePinnedToCore(
        serial_comm_task,
        "task_serial",
        4096,
        NULL,
        3,
        NULL,
        0
    );

    // Core 0: Wi-Fi & Web Server Task (Priority 2)
    xTaskCreatePinnedToCore(
        task_comms,
        "task_comms",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    // Core 1: Real-time Motion Planner & Kinematics (Priority 5, Real-time)
    xTaskCreatePinnedToCore(
        task_motion_planner,
        "task_motion",
        8192,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Firmware boot sequence complete. FreeRTOS Dual-Core operating.");
}
