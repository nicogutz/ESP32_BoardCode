idf_component_register(
        SRCS "nrf.c" "mirf.c" "main.c" "stepper_motor_encoder.c" "wifi.c" "http.c" "bt_server.c"
        INCLUDE_DIRS "."
        REQUIRES esp_driver_gpio esp_timer console driver esp_wifi nvs_flash esp_http_server bt
)
