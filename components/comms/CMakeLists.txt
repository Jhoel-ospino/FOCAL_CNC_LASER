idf_component_register(
    SRCS "wifi_manager.c" "web_server.c" "serial_comm.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event nvs_flash esp_netif esp_http_server esp_driver_uart motion drivers ui freertos log
)
