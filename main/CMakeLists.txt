idf_component_register(
    SRCS 
        "main.c"
        "motors.c"
        "buttons.c"
        "zigbee.c"
        "blinds.c"
        "app_events.c"
        "mcu.c"
        "zcl_utility/src/zcl_utility.c"

    PRIV_REQUIRES 
        "nvs_flash"
        "esp_driver_gpio"
        "esp-zigbee-lib"
        "esp_adc"
        "led_strip"

    INCLUDE_DIRS 
        "."
        "zcl_utility/include"

)

set(COMPONENT_KCONFIG Kconfig)
