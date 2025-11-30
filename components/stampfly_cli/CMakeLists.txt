idf_component_register(
    SRCS "cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_system stampfly_state esp_vfs_console vfs
)
