/**
 * @file gcode_parser.c
 * @brief GRBL 1.1 Compatible G-code Parser and Command Processor
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "gcode_parser.h"
#include "planner.h"
#include "stepper_timer.h"
#include "laser_driver.h"
#include "buzzer_driver.h"
#include "esp_log.h"

static const char *TAG = "GCODE";

static coord_mode_t s_coord_mode = COORD_MODE_ABSOLUTE;
static float s_feedrate = 1200.0f; // mm/min
static uint16_t s_laser_power = 0; // 0..1000
static bool s_dynamic_laser = true; // $32=1 mode M4 default for laser cutter

void gcode_parser_init(void)
{
    s_coord_mode = COORD_MODE_ABSOLUTE;
    s_feedrate = 1200.0f;
    s_laser_power = 0;
    s_dynamic_laser = true;
    ESP_LOGI(TAG, "GRBL 1.1 G-code Parser inicializado.");
}

void gcode_parser_get_status_report(char *buf, size_t max_len)
{
    float cur_x = 0.0f, cur_y = 0.0f;
    planner_get_position(&cur_x, &cur_y);

    const char *state_str = "Idle";
    switch (stepper_timer_get_state()) {
        case CNC_STATE_RUN:   state_str = "Run"; break;
        case CNC_STATE_HOLD:  state_str = "Hold:0"; break;
        case CNC_STATE_ALARM: state_str = "Alarm"; break;
        default:              state_str = "Idle"; break;
    }

    uint32_t buf_avail = PLANNER_BUFFER_SIZE - planner_get_block_count();

    // Standard GRBL 1.1 format: <Idle|WPos:120.000,85.000,0.000|Bf:31,128|FS:1200,500>
    snprintf(buf, max_len, "<%s|WPos:%.3f,%.3f,0.000|Bf:%lu,128|FS:%.0f,%u|Pn:P>\r\n",
             state_str, cur_x, cur_y, (unsigned long)buf_avail, s_feedrate, s_laser_power);
}

bool gcode_parser_execute_line(const char *line, char *response_buf, size_t max_resp_len)
{
    if (!line || strlen(line) == 0) {
        if (response_buf) snprintf(response_buf, max_resp_len, "ok\r\n");
        return true;
    }

    // Trim leading whitespace
    while (isspace((unsigned char)*line)) line++;

    // Real-time Single Character Commands
    if (*line == '?') {
        gcode_parser_get_status_report(response_buf, max_resp_len);
        return true;
    }
    if (*line == '~') { // Resume
        stepper_timer_start();
        if (response_buf) snprintf(response_buf, max_resp_len, "ok\r\n");
        return true;
    }
    if (*line == '!') { // Feed Hold
        stepper_timer_stop();
        if (response_buf) snprintf(response_buf, max_resp_len, "ok\r\n");
        return true;
    }
    if ((uint8_t)*line == 0x18) { // Soft Reset (Ctrl+X)
        planner_abort();
        laser_set_power(0);
        stepper_timer_set_state(CNC_STATE_IDLE);
        if (response_buf) snprintf(response_buf, max_resp_len, "\r\nGrbl 1.1h ['$' for help]\r\n");
        return true;
    }

    // GRBL System Commands
    if (strncmp(line, "$H", 2) == 0) { // Homing Virtual (Modo seguro de pruebas)
        planner_set_position(0.0f, 0.0f);
        ESP_LOGI(TAG, "Homing virtual ejecutado -> Origen (0.0, 0.0) fijado.");
        if (response_buf) snprintf(response_buf, max_resp_len, "ok\r\n");
        return true;
    }

    // Jogging Command: $J=G91 G21 X... Y... F...
    bool is_jog = false;
    coord_mode_t saved_coord_mode = s_coord_mode;
    if (strncmp(line, "$J=", 3) == 0) {
        line += 3;
        is_jog = true;
    }

    // Standard G-code parsing
    float target_x = 0.0f, target_y = 0.0f;
    planner_get_position(&target_x, &target_y);

    bool has_x = false;
    bool has_y = false;
    bool is_motion = is_jog; // Jog commands are intrinsically motion commands

    const char *ptr = line;
    while (*ptr != '\0') {
        while (isspace((unsigned char)*ptr)) ptr++;
        if (*ptr == '\0') break;

        char cmd = toupper((unsigned char)*ptr++);
        char *endptr = NULL;
        float val = strtof(ptr, &endptr);
        ptr = endptr;

        switch (cmd) {
            case 'G':
                if ((int)val == 0 || (int)val == 1) {
                    is_motion = true;
                } else if ((int)val == 90) {
                    s_coord_mode = COORD_MODE_ABSOLUTE;
                } else if ((int)val == 91) {
                    s_coord_mode = COORD_MODE_RELATIVE;
                } else if ((int)val == 92) {
                    // Set Virtual Origin G92
                    planner_set_position(0.0f, 0.0f);
                    ESP_LOGI(TAG, "G92: Origen virtual fijado en (0.0, 0.0)");
                }
                break;
            case 'M':
                if ((int)val == 2 || (int)val == 30) {
                    // Fin de trabajo / programa
                    laser_set_power(0);
                    buzzer_play_job_complete();
                    ESP_LOGI(TAG, "M%d: Fin de trabajo G-code.", (int)val);
                } else if ((int)val == 3) {
                    s_dynamic_laser = false;
                    laser_set_power(s_laser_power);
                } else if ((int)val == 4) {
                    s_dynamic_laser = true;
                } else if ((int)val == 5) {
                    s_laser_power = 0;
                    laser_set_power(0);
                }
                break;
            case 'X':
                has_x = true;
                if (s_coord_mode == COORD_MODE_ABSOLUTE) target_x = val;
                else target_x += val;
                break;
            case 'Y':
                has_y = true;
                if (s_coord_mode == COORD_MODE_ABSOLUTE) target_y = val;
                else target_y += val;
                break;
            case 'F':
                if (val > 0.0f) s_feedrate = val;
                break;
            case 'S':
                if (val >= 0.0f) {
                    s_laser_power = (val > 1000.0f) ? 1000 : (uint16_t)val;
                    if (!s_dynamic_laser) laser_set_power(s_laser_power);
                }
                break;
            default:
                break;
        }
    }

    if (is_jog) {
        s_coord_mode = saved_coord_mode; // Restore modal coordinate mode after jog
    }

    if (is_motion && (has_x || has_y)) {
        ESP_LOGI(TAG, "Encolando movimiento -> Target: (%.2f, %.2f) F: %.0f mm/min (Jog=%d)", 
                 target_x, target_y, s_feedrate, is_jog);
        bool queued = planner_buffer_line(target_x, target_y, s_feedrate, s_laser_power, s_dynamic_laser);
        if (queued) {
            stepper_timer_start();
        } else {
            ESP_LOGW(TAG, "Buffer de movimiento lleno, no se pudo encolar movimiento.");
        }
    }

    if (response_buf) {
        snprintf(response_buf, max_resp_len, "ok\r\n");
    }
    return true;
}
