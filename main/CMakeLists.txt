idf_component_register(SRCS "i2c_oled_example_main.c" "lvgl_demo_ui.c" "sensor.c" "ethernet.c" "wifi.c" "mqtt.c" "fan_control.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_ledc nvs_flash esp_wifi esp_netif esp_eth esp_event)




