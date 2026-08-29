idf_component_register(
    SRC_DIRS
        "system"
        "devices"
        "drivers"
        "drivers/bus"
        "drivers/foc"
    EXCLUDE_SRCS
        "devices/mpu6050_dev.cpp"
        "drivers/mpu6050.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_adc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_mcpwm
        esp_driver_uart
        esp_rom
        esp_system
        esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    "-mtext-section-literals"
)
