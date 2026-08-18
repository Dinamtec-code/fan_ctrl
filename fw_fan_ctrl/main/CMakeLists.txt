idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES hmi_display usb_tmc nvs_flash esp_driver_ledc esp_driver_gpio)

