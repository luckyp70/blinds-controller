idf_component_register(
    SRCS 
        "main.c"
        "motors.c"
        "buttons.c"
        "zigbee.c"

    PRIV_REQUIRES 
        "spi_flash"
        "esp_driver_gpio"
        
    INCLUDE_DIRS "."
)
