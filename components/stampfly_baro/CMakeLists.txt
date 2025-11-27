idf_component_register(
    SRCS "src/bmp280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
