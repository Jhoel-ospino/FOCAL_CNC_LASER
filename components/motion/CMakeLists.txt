idf_component_register(
    SRCS "planner.c" "stepper_timer.c" "gcode_parser.c" "gcode_streamer.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer esp_driver_gpio drivers freertos log
)
