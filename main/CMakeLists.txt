idf_component_register(SRCS "main.c" "oled_draw.c"
                       PRIV_REQUIRES
                           spi_flash
                           esp_driver_ledc
                           esp_driver_i2c
                           esp_wifi
                           nvs_flash
                           esp_event
                           esp_netif
                           freertos
                           log
                           esp_hw_support
                       INCLUDE_DIRS "")
