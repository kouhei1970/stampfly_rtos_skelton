idf_component_register(
    SRCS "src/stampfly_state.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos nvs_flash esp_timer
)
