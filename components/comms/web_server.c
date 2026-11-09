/**
 * @file web_server.c
 * @brief Embedded REST HTTP Server and Focal Web UI
 */

#include <stdio.h>
#include <string.h>
#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "planner.h"
#include "stepper_timer.h"
#include "gcode_parser.h"
#include "sd_card.h"

static const char *TAG = "WEB_SERVER";
static httpd_handle_t s_server = NULL;

// Embedded HTML5 / CSS3 / Vanilla JS Web UI implementing focal-design-system
static const char INDEX_HTML[] = 
"<!DOCTYPE html>\n"
"<html lang='es'>\n"
"<head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>\n"
"<title>Focal CNC Laser 400x400</title>\n"
"<style>\n"
":root {\n"
"  --bg-app: #0F1115;\n"
"  --bg-card: #1A1D24;\n"
"  --bg-card-hover: #222630;\n"
"  --bg-subtle: #262A35;\n"
"  --border-subtle: #2D333F;\n"
"  --pastel-blue: #60A5FA;\n"
"  --pastel-mint: #34D399;\n"
"  --pastel-amber: #FBBF24;\n"
"  --pastel-red: #F87171;\n"
"  --text-primary: #F9FAFB;\n"
"  --text-secondary: #9CA3AF;\n"
"  --text-tertiary: #6B7280;\n"
"  --radius-m: 14px;\n"
"  --radius-l: 20px;\n"
"  --radius-pill: 999px;\n"
"}\n"
"* { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Inter', 'Segoe UI', sans-serif; }\n"
"body { background: var(--bg-app); color: var(--text-primary); padding: 24px; display: flex; justify-content: center; min-height: 100vh; }\n"
".container { width: 100%; max-width: 960px; display: flex; flex-direction: column; gap: 20px; }\n"
"header { display: flex; justify-content: space-between; align-items: center; padding-bottom: 12px; border-bottom: 1px solid var(--border-subtle); }\n"
".brand { font-size: 20px; font-weight: 700; color: var(--text-primary); letter-spacing: -0.5px; }\n"
".brand span { color: var(--pastel-blue); }\n"
".pill { display: inline-flex; align-items: center; gap: 8px; padding: 6px 14px; border-radius: var(--radius-pill); background: var(--bg-subtle); border: 1px solid var(--border-subtle); font-size: 12px; font-weight: 600; }\n"
".pip { width: 8px; height: 8px; border-radius: 50%; background: var(--pastel-mint); }\n"
".grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }\n"
"@media(max-width: 720px) { .grid { grid-template-columns: 1fr; } }\n"
".card { background: var(--bg-card); border: 1px solid var(--border-subtle); border-radius: var(--radius-l); padding: 20px; }\n"
".card h3 { font-size: 14px; color: var(--text-secondary); text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 14px; }\n"
".dro-value { font-size: 28px; font-weight: 700; color: var(--pastel-blue); margin-bottom: 6px; }\n"
".jog-pad { display: grid; grid-template-columns: repeat(3, 64px); gap: 10px; justify-content: center; margin: 16px 0; }\n"
".btn { background: var(--bg-subtle); border: 1px solid var(--border-subtle); border-radius: var(--radius-m); color: var(--text-primary); padding: 10px 16px; font-size: 13px; font-weight: 600; cursor: pointer; transition: all 0.12s ease; text-align: center; }\n"
".btn:hover { background: var(--bg-card-hover); border-color: var(--pastel-blue); }\n"
".btn-mint { background: var(--pastel-mint); color: #0F1115; border: none; }\n"
".btn-mint:hover { opacity: 0.9; }\n"
".btn-danger { background: var(--pastel-red); color: #0F1115; border: none; }\n"
".btn-amber { background: var(--pastel-amber); color: #0F1115; border: none; }\n"
".terminal { display: flex; gap: 10px; margin-top: 14px; }\n"
".terminal input { flex: 1; background: var(--bg-app); border: 1px solid var(--border-subtle); border-radius: var(--radius-m); padding: 10px 14px; color: var(--text-primary); outline: none; }\n"
".terminal input:focus { border-color: var(--pastel-blue); }\n"
".progress-bar { width: 100%; height: 16px; background: var(--bg-subtle); border-radius: var(--radius-pill); overflow: hidden; margin-top: 12px; }\n"
".progress-fill { width: 30%; height: 100%; background: var(--pastel-blue); border-radius: var(--radius-pill); transition: width 0.3s ease; }\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='container'>\n"
"  <header>\n"
"    <div class='brand'>Focal <span>Laser 400x400</span></div>\n"
"    <div class='pill'><div class='pip' id='status-pip'></div><span id='status-text'>CONECTADO</span></div>\n"
"  </header>\n"
"  <div class='grid'>\n"
"    <div class='card'>\n"
"      <h3>Coordenadas de Trabajo (DRO)</h3>\n"
"      <div class='dro-value' id='dro-x'>X: +0.000 mm</div>\n"
"      <div class='dro-value' id='dro-y'>Y: +0.000 mm</div>\n"
"      <div style='color: var(--text-tertiary); font-size: 12px; margin-top: 8px;' id='dro-info'>Husillos 4-start (400 stp/mm)</div>\n"
"      <div style='display:flex; gap: 8px; margin-top: 16px;'>\n"
"        <button class='btn' onclick=\"sendGCode('G92 X0 Y0')\">Fijar Cero</button>\n"
"        <button class='btn' onclick=\"sendGCode('G0 X0 Y0')\">Ir al Cero</button>\n"
"        <button class='btn btn-amber' onclick=\"sendGCode('M3 S10; G4 P1; M5')\">Prueba 1%</button>\n"
"      </div>\n"
"    </div>\n"
"    <div class='card'>\n"
"      <h3>Jogging Direccional</h3>\n"
"      <div class='jog-pad'>\n"
"        <div></div><button class='btn' onclick=\"jog('Y', 10)\">▲ Y+</button><div></div>\n"
"        <button class='btn' onclick=\"jog('X', -10)\">◀ X-</button><button class='btn' onclick=\"sendGCode('$H')\">HOME</button><button class='btn' onclick=\"jog('X', 10)\">▶ X+</button>\n"
"        <div></div><button class='btn' onclick=\"jog('Y', -10)\">▼ Y-</button><div></div>\n"
"      </div>\n"
"      <div style='display:flex; justify-content:center; gap: 8px;'>\n"
"        <button class='btn' onclick=\"setStep(0.1)\">0.1 mm</button>\n"
"        <button class='btn' onclick=\"setStep(1.0)\">1.0 mm</button>\n"
"        <button class='btn' style='border-color: var(--pastel-blue)' onclick=\"setStep(10)\">10 mm</button>\n"
"        <button class='btn' onclick=\"setStep(50)\">50 mm</button>\n"
"      </div>\n"
"    </div>\n"
"  </div>\n"
"  <div class='card'>\n"
"    <h3>Consola Directa G-code (GRBL 1.1)</h3>\n"
"    <div class='terminal'>\n"
"      <input type='text' id='gcode-input' placeholder='Ingresar comando (ej. G1 X100 Y50 F1200 S600, ? o $H)...' onkeydown='if(event.key===\"Enter\") sendInput()'>\n"
"      <button class='btn btn-mint' onclick='sendInput()'>Enviar</button>\n"
"      <button class='btn btn-danger' onclick=\"sendGCode('!')\">PARADA EMERGENCIA</button>\n"
"    </div>\n"
"  </div>\n"
"</div>\n"
"<script>\n"
"let currentStep = 10;\n"
"function setStep(s) { currentStep = s; }\n"
"function jog(axis, dist) {\n"
"  let target = (dist > 0) ? currentStep : -currentStep;\n"
"  sendGCode('$J=G91 G21 ' + axis + target + ' F1200');\n"
"}\n"
"function sendGCode(cmd) {\n"
"  fetch('/command', { method: 'POST', body: cmd })\n"
"    .then(r => r.text())\n"
"    .then(t => console.log('Resp:', t));\n"
"}\n"
"function sendInput() {\n"
"  let el = document.getElementById('gcode-input');\n"
"  if (!el.value.trim()) return;\n"
"  sendGCode(el.value.trim());\n"
"  el.value = '';\n"
"}\n"
"setInterval(() => {\n"
"  fetch('/status')\n"
"    .then(r => r.json())\n"
"    .then(d => {\n"
"      document.getElementById('dro-x').innerText = 'X: ' + (d.wx >= 0 ? '+' : '') + d.wx.toFixed(3) + ' mm';\n"
"      document.getElementById('dro-y').innerText = 'Y: ' + (d.wy >= 0 ? '+' : '') + d.wy.toFixed(3) + ' mm';\n"
"      document.getElementById('status-text').innerText = d.state.toUpperCase();\n"
"    }).catch(e => console.log(e));\n"
"}, 600);\n"
"</script>\n"
"</body>\n"
"</html>\n";

// =============================================================================
// HTTP Handlers
// =============================================================================
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    float cur_x = 0.0f, cur_y = 0.0f;
    planner_get_position(&cur_x, &cur_y);

    const char *state_str = "Idle";
    switch (stepper_timer_get_state()) {
        case CNC_STATE_RUN:   state_str = "Run"; break;
        case CNC_STATE_HOLD:  state_str = "Hold"; break;
        case CNC_STATE_ALARM: state_str = "Alarm"; break;
        default:              state_str = "Idle"; break;
    }

    uint32_t buf_avail = PLANNER_BUFFER_SIZE - planner_get_block_count();

    char json_buf[192];
    snprintf(json_buf, sizeof(json_buf),
             "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":0.000,\"feed\":1200,\"laser\":0,\"buffer\":%lu}",
             state_str, cur_x, cur_y, buf_avail);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t command_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    char resp[128];
    gcode_parser_execute_line(buf, resp, sizeof(resp));

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t files_get_handler(httpd_req_t *req)
{
    char files_json[1024];
    sd_card_list_files_json(files_json, sizeof(files_json));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, files_json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP Web Server on port %d...", config.server_port);
    if (httpd_start(&s_server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(s_server, &uri_root);

        httpd_uri_t uri_status = { .uri = "/status", .method = HTTP_GET, .handler = status_get_handler };
        httpd_register_uri_handler(s_server, &uri_status);

        httpd_uri_t uri_command = { .uri = "/command", .method = HTTP_POST, .handler = command_post_handler };
        httpd_register_uri_handler(s_server, &uri_command);

        httpd_uri_t uri_files = { .uri = "/files", .method = HTTP_GET, .handler = files_get_handler };
        httpd_register_uri_handler(s_server, &uri_files);

        ESP_LOGI(TAG, "HTTP Web Server started successfully.");
    }
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}
