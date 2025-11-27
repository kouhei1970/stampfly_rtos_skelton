idf_component_register(
    SRCS "bmi270_wrapper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
