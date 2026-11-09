/**
 * @file xpt2046.c
 * @brief Driver para panel táctil resistivo XPT2046 con filtrado besttwoavg,
 *        suavizado exponencial (EMA), debounce de liberación y sincronización SPI con ILI9488.
 */

#include "xpt2046.h"
#include "pinout.h"
#include "ili9488_lvgl.h"
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"

static const char *TAG = "XPT2046";

// Comandos XPT2046 en modo diferencial 12-bit
// 0xB1: Presión Z1 (ADC ON)
// 0xC1: Presión Z2 (ADC ON)
// 0x91: Canal Y en sensor resistivo / Dimensión Larga (ADC ON)
// 0xD1: Canal X en sensor resistivo / Dimensión Corta (ADC ON)
// 0xD0: Última lectura con apagado de ADC (PD1=0, PD0=0) para reactivar PENIRQ
#define CMD_READ_Z1          0xB1
#define CMD_READ_Z2          0xC1
#define CMD_READ_Y           0x91
#define CMD_READ_X           0xD1
#define CMD_POWER_DOWN       0xD0

#define TOUCH_PRESSURE_MIN   50
#define TOUCH_RAW_X_MIN      250
#define TOUCH_RAW_X_MAX      3800
#define TOUCH_RAW_Y_MIN      250
#define TOUCH_RAW_Y_MAX      3800

#define LCD_WIDTH            480
#define LCD_HEIGHT           320

static spi_device_handle_t s_touch_spi = NULL;
static lv_indev_t *s_indev = NULL;

// Variables dinámicas de calibración
// En landscape 480x320 para ILI9488 3.5":
// swap_xy = true (Touch Y [long] -> Screen X, Touch X [short] -> Screen Y)
// invert_x = true, invert_y = true
static bool s_swap_xy = true;
static bool s_invert_x = true;
static bool s_invert_y = true;

// Variables de diagnóstico
static uint16_t s_last_raw_x = 0;
static uint16_t s_last_raw_y = 0;
static uint16_t s_last_raw_z = 0;
static int s_last_sx = 0;
static int s_last_sy = 0;
static bool s_last_touched = false;

// Estado de filtro de suavizado
static int32_t s_smooth_x = -1;
static int32_t s_smooth_y = -1;
static int s_release_debounce = 0;

static uint16_t xpt2046_read_raw_channel(uint8_t cmd)
{
    if (!s_touch_spi) return 0;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 24; // 8 bits comando + 16 bits lectura ADC
    t.tx_data[0] = cmd;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = 0x00;

    esp_err_t ret = spi_device_polling_transmit(s_touch_spi, &t);
    if (ret != ESP_OK) return 0;

    // El resultado de 12 bits viene en los bits 14..3 de rx_data[1..2]
    uint16_t raw_val = ((t.rx_data[1] << 8) | t.rx_data[2]) >> 3;
    return raw_val & 0x0FFF;
}

/**
 * @brief Algoritmo besttwoavg:
 * Compara distancias entre 3 muestras y promedia las dos más cercanas,
 * rechazando transitorios sin introducir retraso.
 */
static int32_t besttwoavg(int32_t a, int32_t b, int32_t c)
{
    int32_t da = abs(a - b);
    int32_t db = abs(a - c);
    int32_t dc = abs(b - c);
    if (da <= db && da <= dc) return (a + b) / 2;
    else if (db <= da && db <= dc) return (a + c) / 2;
    else return (b + c) / 2;
}

bool xpt2046_get_touch(int *out_screen_x, int *out_screen_y, uint16_t *out_z)
{
    // 0. Sincronización del bus SPI compartido: asegurar que el LCD DMA esté ocioso
    ili9488_wait_idle();

    // 1. Verificación por hardware con pin T_IRQ
    int irq_val = gpio_get_level(PIN_NUM_TOUCH_IRQ);

    // 2. Medir presión de contacto Z1
    xpt2046_read_raw_channel(CMD_READ_Z1);
    uint16_t z1 = xpt2046_read_raw_channel(CMD_READ_Z1);

    // Si el pin T_IRQ está libre (1) Y la presión es baja, está liberado
    if (irq_val == 1 && z1 < TOUCH_PRESSURE_MIN) {
        if (s_last_touched && s_release_debounce < 3) {
            s_release_debounce++;
            if (out_screen_x) *out_screen_x = s_last_sx;
            if (out_screen_y) *out_screen_y = s_last_sy;
            if (out_z) *out_z = s_last_raw_z;
            return true;
        }
        s_release_debounce = 0;
        s_smooth_x = -1;
        s_smooth_y = -1;
        s_last_touched = false;
        xpt2046_read_raw_channel(CMD_POWER_DOWN);
        return false;
    }

    s_release_debounce = 0;

    // 3. Adquisición de muestras analógicas
    // Canal Y (dimensión larga): descartar 1ra lectura de asentamiento, adquirir 3
    xpt2046_read_raw_channel(CMD_READ_Y);
    int32_t y0 = xpt2046_read_raw_channel(CMD_READ_Y);
    int32_t y1 = xpt2046_read_raw_channel(CMD_READ_Y);
    int32_t y2 = xpt2046_read_raw_channel(CMD_READ_Y);

    // Canal X (dimensión corta): descartar 1ra lectura de asentamiento, adquirir 3
    xpt2046_read_raw_channel(CMD_READ_X);
    int32_t x0 = xpt2046_read_raw_channel(CMD_READ_X);
    int32_t x1 = xpt2046_read_raw_channel(CMD_READ_X);
    int32_t x2 = xpt2046_read_raw_channel(CMD_READ_X);

    // Apagar ADC para que el pin T_IRQ quede en modo interrupción sensible a nuevo toque
    xpt2046_read_raw_channel(CMD_POWER_DOWN);

    // Rechazar transitorios extremos si las muestras discrepan enormemente
    if (abs(x0 - x1) > 700 && abs(x1 - x2) > 700) {
        if (s_last_touched) {
            if (out_screen_x) *out_screen_x = s_last_sx;
            if (out_screen_y) *out_screen_y = s_last_sy;
            if (out_z) *out_z = s_last_raw_z;
            return true;
        }
        return false;
    }
    if (abs(y0 - y1) > 700 && abs(y1 - y2) > 700) {
        if (s_last_touched) {
            if (out_screen_x) *out_screen_x = s_last_sx;
            if (out_screen_y) *out_screen_y = s_last_sy;
            if (out_z) *out_z = s_last_raw_z;
            return true;
        }
        return false;
    }

    // 4. Filtrado de ruido por mediana de los dos valores más cercanos (besttwoavg)
    int32_t raw_x = besttwoavg(x0, x1, x2);
    int32_t raw_y = besttwoavg(y0, y1, y2);

    // Rechazar lecturas de circuito abierto fuera de los límites mecánicos del panel
    if (raw_x < 150 || raw_x > 3980 || raw_y < 150 || raw_y > 3980) {
        return false;
    }

    // 5. Filtro IIR balanceado (50% nuevo, 50% anterior para suprimir temblor analógico)
    if (s_smooth_x < 0) {
        s_smooth_x = raw_x;
        s_smooth_y = raw_y;
    } else {
        s_smooth_x = (s_smooth_x * 50 + raw_x * 50) / 100;
        s_smooth_y = (s_smooth_y * 50 + raw_y * 50) / 100;
    }

    // 6. Mapeo de coordenadas con calibración dinámica
    int32_t tx = s_swap_xy ? s_smooth_y : s_smooth_x;
    int32_t ty = s_swap_xy ? s_smooth_x : s_smooth_y;

    int sx = (tx - TOUCH_RAW_X_MIN) * (LCD_WIDTH - 1) / (TOUCH_RAW_X_MAX - TOUCH_RAW_X_MIN);
    int sy = (ty - TOUCH_RAW_Y_MIN) * (LCD_HEIGHT - 1) / (TOUCH_RAW_Y_MAX - TOUCH_RAW_Y_MIN);

    if (s_invert_x) sx = (LCD_WIDTH - 1) - sx;
    if (s_invert_y) sy = (LCD_HEIGHT - 1) - sy;

    // Clamping de seguridad dentro de los límites de la pantalla
    if (sx < 0) sx = 0;
    if (sx >= LCD_WIDTH) sx = LCD_WIDTH - 1;
    if (sy < 0) sy = 0;
    if (sy >= LCD_HEIGHT) sy = LCD_HEIGHT - 1;

    // 7. Zona muerta (Deadband): Si el contacto se sostiene, anular jitter <= 3 px
    if (s_last_touched) {
        int dx = abs(sx - s_last_sx);
        int dy = abs(sy - s_last_sy);
        if (dx <= 3 && dy <= 3) {
            sx = s_last_sx;
            sy = s_last_sy;
        }
    }

    s_last_raw_x = (uint16_t)s_smooth_x;
    s_last_raw_y = (uint16_t)s_smooth_y;
    s_last_raw_z = z1;
    s_last_sx = sx;
    s_last_sy = sy;
    s_last_touched = true;

    if (out_screen_x) *out_screen_x = sx;
    if (out_screen_y) *out_screen_y = sy;
    if (out_z) *out_z = z1;

    return true;
}

void xpt2046_set_config(bool swap_xy, bool invert_x, bool invert_y)
{
    s_swap_xy = swap_xy;
    s_invert_x = invert_x;
    s_invert_y = invert_y;
    ESP_LOGI(TAG, "Touch Calib actualizada: swap_xy=%d, invert_x=%d, invert_y=%d",
             s_swap_xy, s_invert_x, s_invert_y);
}

void xpt2046_get_config(bool *out_swap_xy, bool *out_invert_x, bool *out_invert_y)
{
    if (out_swap_xy) *out_swap_xy = s_swap_xy;
    if (out_invert_x) *out_invert_x = s_invert_x;
    if (out_invert_y) *out_invert_y = s_invert_y;
}

bool xpt2046_get_last_debug(uint16_t *out_raw_x, uint16_t *out_raw_y, uint16_t *out_z, int *out_sx, int *out_sy)
{
    if (out_raw_x) *out_raw_x = s_last_raw_x;
    if (out_raw_y) *out_raw_y = s_last_raw_y;
    if (out_z) *out_z = s_last_raw_z;
    if (out_sx) *out_sx = s_last_sx;
    if (out_sy) *out_sy = s_last_sy;
    return s_last_touched;
}

static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    int screen_x = 0;
    int screen_y = 0;
    uint16_t z = 0;

    if (xpt2046_get_touch(&screen_x, &screen_y, &z)) {
        data->point.x = (lv_coord_t)screen_x;
        data->point.y = (lv_coord_t)screen_y;
        data->state = LV_INDEV_STATE_PR;

        static int s_log_count = 0;
        if (++s_log_count % 10 == 0) {
            ESP_LOGI(TAG, "TOUCH: Raw(X=%u, Y=%u, Z=%u) -> Screen(%d, %d)",
                     s_last_raw_x, s_last_raw_y, z, screen_x, screen_y);
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

esp_err_t xpt2046_init(void)
{
    ESP_LOGI(TAG, "Configurando panel táctil XPT2046 (CS=%d, IRQ=%d, SPI=2MHz)...",
             PIN_NUM_TOUCH_CS, PIN_NUM_TOUCH_IRQ);

    // 1. Configurar pin de interrupción T_IRQ con pull-up
    gpio_config_t irq_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_TOUCH_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&irq_conf);

    // 2. Agregar dispositivo SPI en el bus SPI2 compartido a 2 MHz para alta estabilidad
    spi_device_interface_config_t touch_devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_TOUCH_CS,
        .queue_size = 2,
    };

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &touch_devcfg, &s_touch_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando dispositivo touch XPT2046 al bus SPI2 (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Dispositivo XPT2046 registrado con éxito en SPI2.");
    return ESP_OK;
}

void xpt2046_register_lvgl_indev(void)
{
    ESP_LOGI(TAG, "Registrando dispositivo de entrada táctil (indev) en LVGL...");
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_read_cb;
    indev_drv.scroll_limit = 10;            // Requiere 10px de movimiento intencional antes de activar scroll (elimina falsos scrolls por ruido resistivo)
    indev_drv.scroll_throw = 20;            // Desaceleracion de inercia suave (%)
    indev_drv.long_press_time = 450;        // 450 ms para toque sostenido libre de falsos positivos
    indev_drv.gesture_limit = 25;           // Deteccion de gestos de deslizamiento
    indev_drv.gesture_min_velocity = 3;
    s_indev = lv_indev_drv_register(&indev_drv);

    if (s_indev) {
        ESP_LOGI(TAG, "Driver táctil registrado correctamente en LVGL.");
    } else {
        ESP_LOGE(TAG, "Fallo al registrar driver táctil en LVGL.");
    }
}
