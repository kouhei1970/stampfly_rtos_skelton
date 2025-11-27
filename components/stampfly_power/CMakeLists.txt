idf_component_register(
    SRCS "src/power_monitor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
