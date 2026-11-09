idf_component_register(
    SRCS "xpt2046.c" "ili9488_lvgl.c" "laser_driver.c" "buzzer_driver.c" "sd_card.c" "tmc2209.c" "settings_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_spi esp_driver_uart esp_driver_ledc esp_timer esp_lcd fatfs sdmmc lvgl freertos nvs_flash wear_levelling
    EMBED_TXTFILES "prueba_mario.gcode"
)
