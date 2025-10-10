idf_component_register(SRCS "ssd1306.c" "Tramkiemsoat.c" "dfplayer.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           esp_driver_gpio
                           nvs_flash
                           esp_netif
                           esp_wifi 
                           mqtt
                           esp_timer
                           esp_driver_spi
                           esp_driver_uart
                           esp_driver_i2c
                           log
                           spi_flash
)

target_add_binary_data(${COMPONENT_TARGET} "certs/hivemq_isrg_root_x1.pem" TEXT)
