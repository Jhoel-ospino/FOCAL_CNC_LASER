/**
 * @file ili9488_lvgl.c
 * @brief Implementación del driver de pantalla ILI9488 con LVGL v8 y esp_lcd.
 */

#include "ili9488_lvgl.h"
#include "pinout.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_rom_sys.h"
#include "lvgl.h"
#include "xpt2046.h"

static const char *TAG = "ILI9488_LVGL";

#define LCD_WIDTH           480
#define LCD_HEIGHT          320
#define LVGL_BUFFER_LINES   40
#define LVGL_BUFFER_PIXELS  (LCD_WIDTH * LVGL_BUFFER_LINES)

static esp_lcd_panel_io_handle_t s_io_handle = NULL;
static SemaphoreHandle_t s_lvgl_mutex = NULL;
static uint8_t *s_dma_tx_buf = NULL;

static lv_disp_draw_buf_t s_disp_buf;
static lv_color_t *s_buf1 = NULL;
static lv_color_t *s_buf2 = NULL;
static lv_disp_drv_t s_disp_drv;
static volatile bool s_lcd_dma_busy = false;

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    s_lcd_dma_busy = false;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lv_tick_task(void *arg)
{
    lv_tick_inc(2);
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    uint16_t x0 = area->x1;
    uint16_t y0 = area->y1;
    uint16_t x1 = area->x2;
    uint16_t y1 = area->y2;
    uint16_t w = x1 - x0 + 1;
    uint16_t h = y1 - y0 + 1;
    uint32_t total_pixels = (uint32_t)w * h;

    // 1. Esperar a que cualquier ráfaga DMA previa haya terminado de forma segura
    ili9488_wait_idle();

    // 2. Conversión optimizada de color RGB565 a RGB666 mediante punteros sin multiplicaciones
    uint8_t *dst = s_dma_tx_buf;
    const lv_color_t *src = color_map;
    const lv_color_t *src_end = color_map + total_pixels;
    while (src < src_end) {
        uint16_t c = src->full;
        *dst++ = (uint8_t)((c >> 11) << 3);
        *dst++ = (uint8_t)(((c >> 5) & 0x3F) << 2);
        *dst++ = (uint8_t)((c & 0x1F) << 3);
        src++;
    }

    // 3. Column Address Set (0x2A)
    uint8_t d_col[4] = { (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF) };
    esp_lcd_panel_io_tx_param(s_io_handle, 0x2A, d_col, 4);

    // 4. Page Address Set (0x2B)
    uint8_t d_row[4] = { (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF) };
    esp_lcd_panel_io_tx_param(s_io_handle, 0x2B, d_row, 4);

    // 5. Memory Write (0x2C) con ráfaga DMA asíncrona.
    // Al concluir la ráfaga por hardware, el callback notify_lvgl_flush_ready()
    // llamará automáticamente a lv_disp_flush_ready(drv).
    s_lcd_dma_busy = true;
    esp_lcd_panel_io_tx_color(s_io_handle, 0x2C, s_dma_tx_buf, total_pixels * 3);
}

static void ili9488_init_registers(void)
{
    gpio_set_level(PIN_NUM_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(PIN_NUM_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 0xE0: Positive Gamma Control
    const uint8_t d_e0[] = { 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xE0, d_e0, sizeof(d_e0));

    // 0xE1: Negative Gamma Control
    const uint8_t d_e1[] = { 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xE1, d_e1, sizeof(d_e1));

    // 0xC0: Power Control 1
    const uint8_t d_c0[] = { 0x17, 0x15 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xC0, d_c0, sizeof(d_c0));

    // 0xC1: Power Control 2
    const uint8_t d_c1[] = { 0x41 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xC1, d_c1, sizeof(d_c1));

    // 0xC5: VCOM Control
    const uint8_t d_c5[] = { 0x00, 0x12, 0x80 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xC5, d_c5, sizeof(d_c5));

    // 0x36: MADCTL (Landscape 480x320 BGR)
    const uint8_t d_36[] = { 0x28 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0x36, d_36, sizeof(d_36));

    // 0x3A: Interface Pixel Format (0x66 = 18-bit RGB666)
    const uint8_t d_3a[] = { 0x66 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0x3A, d_3a, sizeof(d_3a));

    // 0xB0: Interface Mode Control
    const uint8_t d_b0[] = { 0x00 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xB0, d_b0, sizeof(d_b0));

    // 0xB1: Frame Rate Control (60Hz)
    const uint8_t d_b1[] = { 0xA0 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xB1, d_b1, sizeof(d_b1));

    // 0xB4: Display Inversion Control
    const uint8_t d_b4[] = { 0x02 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xB4, d_b4, sizeof(d_b4));

    // 0xB6: Display Function Control
    const uint8_t d_b6[] = { 0x02, 0x02, 0x3B };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xB6, d_b6, sizeof(d_b6));

    // 0xB7: Entry Mode Set
    const uint8_t d_b7[] = { 0xC6 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xB7, d_b7, sizeof(d_b7));

    // 0xF7: Adjust Control 3
    const uint8_t d_f7[] = { 0xA9, 0x51, 0x2C, 0x82 };
    esp_lcd_panel_io_tx_param(s_io_handle, 0xF7, d_f7, sizeof(d_f7));

    // 0x11: Sleep Out
    esp_lcd_panel_io_tx_param(s_io_handle, 0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    // 0x20: Display Inversion Off
    esp_lcd_panel_io_tx_param(s_io_handle, 0x20, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // 0x29: Display On
    esp_lcd_panel_io_tx_param(s_io_handle, 0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
}

esp_err_t ili9488_lvgl_init(void)
{
    ESP_LOGI(TAG, "Inicializando display ILI9488 con puerto LVGL v8...");

    // Mutex de sincronización
    s_lvgl_mutex = xSemaphoreCreateMutex();
    if (!s_lvgl_mutex) return ESP_ERR_NO_MEM;

    // Configurar pin RST
    gpio_reset_pin(PIN_NUM_TFT_RST);
    gpio_set_direction(PIN_NUM_TFT_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_TFT_RST, 1);

    // Asignar buffer DMA para transmisión RGB666
    size_t dma_bytes = LVGL_BUFFER_PIXELS * 3;
    s_dma_tx_buf = (uint8_t *)heap_caps_malloc(dma_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_dma_tx_buf) {
        ESP_LOGE(TAG, "Error asignando memoria DMA para pantalla.");
        return ESP_ERR_NO_MEM;
    }

    // Configuración del bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_TFT_MOSI,
        .miso_io_num = PIN_NUM_TFT_MISO,
        .sclk_io_num = PIN_NUM_TFT_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = dma_bytes,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Error inicializando bus SPI2 (%s)", esp_err_to_name(ret));
        return ret;
    }

    // Panel IO esp_lcd (Aceleración a 40 MHz con cola de transacciones DMA)
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_TFT_DC,
        .cs_gpio_num = PIN_NUM_TFT_CS,
        .pclk_hz = 40 * 1000 * 1000, // 40 MHz ultra-rápido para ILI9488
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &s_disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &s_io_handle));

    // Inicializar registros del ILI9488
    ili9488_init_registers();

    // Inicializar LVGL
    lv_init();

    // Asignar buffers de dibujo de LVGL en memoria interna o PSRAM
    s_buf1 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_PIXELS * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_buf1) {
        s_buf1 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_PIXELS * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    s_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_PIXELS * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_buf2) {
        s_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_PIXELS * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_buf1) {
        ESP_LOGE(TAG, "Error asignando s_buf1 de LVGL");
        return ESP_ERR_NO_MEM;
    }

    lv_disp_draw_buf_init(&s_disp_buf, s_buf1, s_buf2, LVGL_BUFFER_PIXELS);

    // Registrar driver de display en LVGL
    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res = LCD_WIDTH;
    s_disp_drv.ver_res = LCD_HEIGHT;
    s_disp_drv.flush_cb = lvgl_flush_cb;
    s_disp_drv.draw_buf = &s_disp_buf;
    lv_disp_drv_register(&s_disp_drv);

    // Inicializar controlador táctil resistivo XPT2046 en el bus SPI compartido
    xpt2046_init();
    xpt2046_register_lvgl_indev();

    // Timer periódico de tick de LVGL cada 2 ms
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "lv_tick"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2000)); // 2000 us = 2 ms

    ESP_LOGI(TAG, "Puerto LVGL v8 + ILI9488 inicializado con éxito a 480x320.");
    return ESP_OK;
}

bool lvgl_port_lock(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTake(s_lvgl_mutex, ticks) == pdTRUE);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGive(s_lvgl_mutex);
}

void ili9488_wait_idle(void)
{
    int timeout_us = 15000;
    while (s_lcd_dma_busy && timeout_us > 0) {
        esp_rom_delay_us(10);
        timeout_us -= 10;
    }
}
