idf_component_register(SRCS "ssd1306.c" "Tramkiemsoat.c" "dfplayer.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           esp_driver_gpio
                           esp_timer
                           esp_driver_spi
                           esp_driver_uart
                           esp_driver_i2c 
                           log
                           spi_flash
)
