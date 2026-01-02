idf_component_register(
    SRCS "cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_system sf_svc_state esp_vfs_console vfs sf_hal_bmm150 sf_svc_logger sf_svc_comm stampfly_led sf_hal_motor stampfly_buzzer sf_algo_fusion nvs_flash
)
