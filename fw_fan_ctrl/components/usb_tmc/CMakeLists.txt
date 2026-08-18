idf_component_register(
    SRCS 
    "src/usb_descriptors.c" 
    "src/usb_tmc_cb.c" 
    "src/usb_tmc_init.c" 
    "src/usb_tmc_process.c"
    INCLUDE_DIRS "include" 
    REQUIRES tinyusb esp_tinyusb esp_system libscpi controller
    WHOLE_ARCHIVE)

if(TARGET __idf_espressif__tinyusb)
    # 3. Le inyectamos a TinyUSB la ruta hacia la carpeta include de TU componente
    target_include_directories(__idf_espressif__tinyusb PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")

    # 4. Le inyectamos las definiciones de compilación directamente al target
    target_compile_definitions(__idf_espressif__tinyusb PUBLIC 
        CFG_TUD_USBTMC=1
        "CFG_TUSB_CONFIG_FILE=\"usb_tmc_config.h\""
        CFG_TUD_TMC=1
    )
endif()
