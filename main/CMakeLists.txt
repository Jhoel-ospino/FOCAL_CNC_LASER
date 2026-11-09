idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES drivers motion ui comms lvgl freertos esp_timer esp_system log
)
