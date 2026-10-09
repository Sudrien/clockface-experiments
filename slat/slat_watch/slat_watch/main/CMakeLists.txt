idf_component_register(SRCS "slat_watch.c" INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_driver_gpio esp_driver_spi esp_timer esp_wifi esp_netif esp_event nvs_flash)
