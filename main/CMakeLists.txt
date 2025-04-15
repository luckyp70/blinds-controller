idf_component_register(
    SRCS 
        "main.c"
        "motors.c"
        "buttons.c"
        "zigbee.c"
        "zcl_utility/src/zcl_utility.c"

    PRIV_REQUIRES 
        "nvs_flash"
        "esp_driver_gpio"

    INCLUDE_DIRS 
        "."
        "zcl_utility/include"
)

