idf_component_register(
    SRCS "src/led.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
