idf_component_register(
    SRCS "src/bmm150.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
