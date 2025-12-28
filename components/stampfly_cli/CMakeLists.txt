idf_component_register(
    SRCS "cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_system stampfly_state esp_vfs_console vfs stampfly_mag stampfly_logger stampfly_comm stampfly_led stampfly_motor
)
