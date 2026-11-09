/**
 * @file serial_comm.c
 * @brief USB Serial GRBL 1.1 Protocol and Binary File Transfer Handler
 */

#include <stdio.h>
#include <string.h>
#include "serial_comm.h"
#include "gcode_parser.h"
#include "ui_screens.h"
#include "sd_card.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERIAL_COMM";
#define UART_PORT_NUM UART_NUM_0
#define BUF_SIZE      1024

static FILE *s_upload_fp = NULL;
static size_t s_upload_total_bytes = 0;
static size_t s_upload_received_bytes = 0;
static bool s_is_transferring = false;

void serial_comm_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);

    ESP_LOGI(TAG, "Serial communication initialized on UART0 (115200 baud).");
    uart_write_bytes(UART_PORT_NUM, "\r\nGrbl 1.1h ['$' for help]\r\n[MSG:Focal CNC Laser Ready]\r\n", 58);
}

void serial_comm_task(void *pvParameters)
{
    uint8_t data[BUF_SIZE];
    char line_buf[256];
    size_t line_pos = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            char c = (char)data[i];

            // In-stream transfer mode ($FILE_START ... $FILE_END)
            if (s_is_transferring) {
                if (c == '\n' || c == '\r') {
                    line_buf[line_pos] = '\0';
                    if (strncmp(line_buf, "$FILE_END", 9) == 0) {
                        if (s_upload_fp) {
                            fclose(s_upload_fp);
                            s_upload_fp = NULL;
                        }
                        s_is_transferring = false;
                        const char *msg = "\r\n[MSG:File upload complete 100%]\r\nok\r\n";
                        uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
                        line_pos = 0;
                        continue;
                    }
                }
                
                // Write byte to SD file
                if (s_upload_fp) {
                    fputc(c, s_upload_fp);
                    s_upload_received_bytes++;
                    // Progress report every 5%
                    if (s_upload_total_bytes > 0 && (s_upload_received_bytes % (s_upload_total_bytes / 20 + 1)) == 0) {
                        int pct = (int)((s_upload_received_bytes * 100) / s_upload_total_bytes);
                        char pbuf[32];
                        snprintf(pbuf, sizeof(pbuf), "[PROGRESS:%d%%]\r\n", pct);
                        uart_write_bytes(UART_PORT_NUM, pbuf, strlen(pbuf));
                    }
                }
                continue;
            }

            // Real-time single character commands handled immediately
            if (c == '?' || c == '~' || c == '!' || (uint8_t)c == 0x18) {
                char single[2] = {c, '\0'};
                char resp[128];
                gcode_parser_execute_line(single, resp, sizeof(resp));
                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
                continue;
            }

            if (c == '\r' || c == '\n') {
                if (line_pos == 0) continue;
                line_buf[line_pos] = '\0';

                // Check for UI test commands (screen testing without touch)
                if (strncmp(line_buf, "$UI=", 4) == 0) {
                    int scr_id = atoi(line_buf + 4) - 1;
                    if (scr_id >= 0 && scr_id < UI_SCREEN_MAX) {
                        ui_set_demo_mode(false);
                        ui_switch_screen((ui_screen_id_t)scr_id);
                        const char *m = "[MSG:Switched UI Screen]\r\nok\r\n";
                        uart_write_bytes(UART_PORT_NUM, m, strlen(m));
                    }
                    line_pos = 0;
                    continue;
                }
                if (strncmp(line_buf, "$DEMO=1", 7) == 0) {
                    ui_set_demo_mode(true);
                    const char *m = "[MSG:Demo mode enabled]\r\nok\r\n";
                    uart_write_bytes(UART_PORT_NUM, m, strlen(m));
                    line_pos = 0;
                    continue;
                }
                if (strncmp(line_buf, "$DEMO=0", 7) == 0) {
                    ui_set_demo_mode(false);
                    const char *m = "[MSG:Demo mode disabled]\r\nok\r\n";
                    uart_write_bytes(UART_PORT_NUM, m, strlen(m));
                    line_pos = 0;
                    continue;
                }

                // Check for file streaming start: $FILE_START <filename> <size>
                if (strncmp(line_buf, "$FILE_START", 11) == 0) {
                    char fname[64] = {0};
                    size_t fsize = 0;
                    if (sscanf(line_buf + 11, "%63s %zu", fname, &fsize) >= 1) {
                        char path[128];
                        snprintf(path, sizeof(path), "/sdcard/%s", fname);
                        s_upload_fp = fopen(path, "wb");
                        s_upload_total_bytes = fsize;
                        s_upload_received_bytes = 0;
                        s_is_transferring = true;
                        const char *m = "[MSG:File upload started]\r\nok\r\n";
                        uart_write_bytes(UART_PORT_NUM, m, strlen(m));
                    }
                    line_pos = 0;
                    continue;
                }

                // Execute standard G-code line
                char resp[128];
                gcode_parser_execute_line(line_buf, resp, sizeof(resp));
                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));

                line_pos = 0;
            } else {
                if (line_pos < sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = c;
                }
            }
        }
    }
}
