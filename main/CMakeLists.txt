idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES
                           spi_flash
                           esp_driver_gpio
                           esp_driver_ledc
                           esp_wifi
                           nvs_flash
                           esp_event
                           esp_netif
                           freertos
                           log
                           esp_hw_support
                       INCLUDE_DIRS "")
