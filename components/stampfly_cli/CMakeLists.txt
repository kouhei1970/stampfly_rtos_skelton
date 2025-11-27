idf_component_register(
    SRCS "cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_system esp_driver_usb_serial_jtag
)
