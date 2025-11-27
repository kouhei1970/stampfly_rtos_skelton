idf_component_register(
    SRCS "src/controller_comm.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash freertos
)
